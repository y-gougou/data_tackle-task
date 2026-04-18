#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WHEELTEC ROS 底盘串口上位机

发送帧格式 (11字节):
    [0x7B][Mode][保留][Vx_H][Vx_L][Vy_H][Vy_L][Vz_H][Vz_L][BCC][0x7D]

接收帧格式 (24字节):
    [0x7B][FlagStop][Vel_X_H][Vel_X_L][Vel_Y_H][Vel_Y_L][Vel_Z_H][Vel_Z_L]
    [Accel_X_H][Accel_X_L][Accel_Y_H][Accel_Y_L][Accel_Z_H][Accel_Z_L]
    [Gyro_X_H][Gyro_X_L][Gyro_Y_H][Gyro_Y_L][Gyro_Z_H][Gyro_Z_L]
    [Power_H][Power_L][BCC][0x7D]

协议字段说明:
    Vx / Vy / Vz 均为有符号 short、大端、按 1000 倍缩放传输。
    其中线速度通常按 m/s 显示，角速度通常按 rad/s 显示。
    例如 1000 -> 1.000，对应固件中的 value / 1000 还原逻辑。

Accel / Gyro:
    原始有符号 short

Power_Voltage:
    mV, 无符号 short, 大端
"""

from __future__ import annotations

import csv
import os
import queue
import struct
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import filedialog, messagebox, scrolledtext, ttk

import serial
import serial.tools.list_ports


TX_FRAME_LEN = 11
RX_FRAME_LEN = 24
FRAME_HEAD = 0x7B
FRAME_TAIL = 0x7D
LOG_FRAME_HEAD = 0x7E
LOG_FRAME_TAIL = 0x7F
LOG_FRAME_LEN = 19
LOG_TEXT_LEN = 14

BG = "#0d1117"
PANEL = "#161b22"
BORDER = "#30363d"
ACCENT = "#00d4aa"
ACCENT2 = "#ff6b35"
TEXT_PRI = "#e6edf3"
TEXT_SEC = "#8b949e"
TEXT_DIM = "#484f58"
SUCCESS = "#3fb950"
WARNING = "#d29922"
DANGER = "#f85149"
BTN_BG = "#21262d"
PENDING = "#ff9d00"  # 待确认的橙色

CONTROL_MODES = {
    "0: 正常控制": 0x00,
    "1: 自动回充": 0x01,
    "2: 自动回充+导航": 0x02,
    "3: 红外对接": 0x03,
}

LAYOUT_WIDE_MIN = 1280
LAYOUT_COMPACT_MIN = 900

SECTION_LOG = "log"
SECTION_SERIAL = "serial"
SECTION_MOTION = "motion"
SECTION_STATUS = "status"
SECTION_ADVANCED = "advanced"

DESKTOP_SLOT_NAMES = ("top_left", "top_right", "bottom_left", "bottom_right", "bottom_full")


@dataclass(slots=True)
class StatusFrame:
    """解析后的24字节机器人状态帧"""
    flag_stop: int
    vel_x: int
    vel_y: int
    vel_z: int
    accel_x: int
    accel_y: int
    accel_z: int
    gyro_x: int
    gyro_y: int
    gyro_z: int
    power_voltage: int
    bcc_valid: bool

    @classmethod
    def decode(cls, frame: bytes) -> "StatusFrame":
        if len(frame) != RX_FRAME_LEN:
            raise ValueError(f"invalid RX frame length: {len(frame)}")
        bcc_valid = calc_bcc(frame[:22]) == frame[22]
        return cls(
            flag_stop=frame[1],
            vel_x=struct.unpack(">h", frame[2:4])[0],
            vel_y=struct.unpack(">h", frame[4:6])[0],
            vel_z=struct.unpack(">h", frame[6:8])[0],
            accel_x=struct.unpack(">h", frame[8:10])[0],
            accel_y=struct.unpack(">h", frame[10:12])[0],
            accel_z=struct.unpack(">h", frame[12:14])[0],
            gyro_x=struct.unpack(">h", frame[14:16])[0],
            gyro_y=struct.unpack(">h", frame[16:18])[0],
            gyro_z=struct.unpack(">h", frame[18:20])[0],
            power_voltage=struct.unpack(">H", frame[20:22])[0],
            bcc_valid=bcc_valid,
        )


@dataclass(slots=True)
class MotionFrame:
    """发送的11字节控制帧"""
    mode: int
    vx: int
    vy: int
    vz: int

    def encode(self) -> bytes:
        frame = bytearray(TX_FRAME_LEN)
        frame[0] = FRAME_HEAD
        frame[1] = self.mode
        frame[2] = 0x00
        frame[3:5] = struct.pack(">h", self.vx)
        frame[5:7] = struct.pack(">h", self.vy)
        frame[7:9] = struct.pack(">h", self.vz)
        frame[9] = calc_bcc(frame[:9])
        frame[10] = FRAME_TAIL
        return bytes(frame)


@dataclass(slots=True)
class LogFrame:
    code: int
    text: str
    bcc_valid: bool

    @classmethod
    def decode(cls, frame: bytes) -> "LogFrame":
        if len(frame) != LOG_FRAME_LEN:
            raise ValueError(f"invalid LOG frame length: {len(frame)}")
        text_len = min(frame[2], LOG_TEXT_LEN)
        text = frame[3:3 + text_len].decode("ascii", errors="replace").strip("\x00 ")
        bcc_valid = calc_bcc(frame[: LOG_FRAME_LEN - 2]) == frame[LOG_FRAME_LEN - 2]
        return cls(code=frame[1], text=text, bcc_valid=bcc_valid)


def calc_bcc(data: bytes) -> int:
    bcc = 0
    for value in data:
        bcc ^= value
    return bcc & 0xFF


def frame_to_hex(frame: bytes) -> str:
    return " ".join(f"{b:02X}" for b in frame)


def scaled_to_unit(value: int) -> float:
    """固件协议把速度/角速度统一按 1000 倍缩放传输。"""
    return value / 1000.0


class WheeltecController:
    AXIS_CONFIG = (
        ("vx", "Vx 前后", "mm/s", -2000, 2000, 10),
        ("vy", "Vy 左右", "mm/s", -2000, 2000, 10),
        ("vz", "Vz 转向", "rad/s x1000", -3140, 3140, 10),
    )

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("WHEELTEC ROS 底盘串口上位机")
        self.root.configure(bg=BG)
        self.root.minsize(720, 620)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.ser: serial.Serial | None = None
        self.connected = False
        self.read_thread: threading.Thread | None = None
        self.auto_thread: threading.Thread | None = None
        self.log_queue: queue.Queue[tuple[str, str]] = queue.Queue()
        self.rx_buffer = bytearray()
        self.ui_alive = True

        self.vx_var = tk.IntVar(value=0)
        self.vy_var = tk.IntVar(value=0)
        self.vz_var = tk.IntVar(value=0)
        self.step_var = tk.IntVar(value=100)
        self.axis_entry_vars: dict[str, tk.StringVar] = {}

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")
        self.timeout_var = tk.DoubleVar(value=0.10)
        self.auto_interval_var = tk.DoubleVar(value=0.10)
        self.auto_send_delay_var = tk.IntVar(value=80)
        self.mode_var = tk.StringVar(value="0: 正常控制")
        self.auto_reconnect_var = tk.BooleanVar(value=False)
        self.battery_cells_var = tk.StringVar(value="4S")

        self.tx_count_var = tk.StringVar(value="0")
        self.rx_count_var = tk.StringVar(value="0")
        self.last_send_var = tk.StringVar(value="未发送")
        self.rx_state_var = tk.StringVar(value="等待接收")
        self.rx_value_var = tk.StringVar(value="Vx=0.000 m/s  Vy=0.000 m/s  Vz=0.000 rad/s")
        self.battery_var = tk.StringVar(value="-- V")
        self.battery_pct_var = tk.StringVar(value="--%")

        self.auto_send = False
        self.tx_count = 0
        self.rx_count = 0
        self.last_rx_time = "-"
        self.keys_pressed: set[str] = set()
        self.active_pointer_keys: set[str] = set()
        self.log_writer: csv.writer | None = None
        self.log_file: object | None = None

        # 自动发送策略
        self.auto_send_on_change_var = tk.BooleanVar(value=True)
        self.auto_send_job: str | None = None
        self.last_sent_payload: tuple[int, int, int, int] | None = None
        self._mode_change_guard = False

        self.speed_scales: dict[str, tk.Scale] = {}
        self.speed_entries: dict[str, tk.Widget] = {}
        self.speed_labels: dict[str, tk.Label] = {}
        self.byte_labels: list[tk.Label] = []
        self.last_rx_byte_labels: list[tk.Label] = []
        self.status_value_labels: list[tk.Label] = []
        self.frame_bar_groups: list[tuple[tk.Frame, list[tk.Frame]]] = []
        self.section_frames: dict[str, tk.Frame] = {}
        self.section_bodies: dict[str, tk.Frame] = {}
        self.section_order: dict[str, int] = {}
        self.section_headers: dict[str, tk.Frame] = {}
        self.section_drag_labels: dict[str, tk.Label] = {}
        self.section_titles: dict[str, tk.Label] = {}
        self.section_toggles: dict[str, ttk.Button] = {}
        self.collapsed_sections: dict[str, bool] = {}
        self.section_default_border: dict[str, str] = {}
        self.layout_mode = "wide"
        self.layout_orders: dict[str, list[str]] = {}
        self.docking_guides: dict[str, tk.Label] = {}
        self.layout_job: str | None = None
        self.log_poll_job: str | None = None
        self.body_scroll_canvas: tk.Canvas | None = None
        self.body_scroll_frame: tk.Frame | None = None
        self.body_scroll_window: int | None = None
        self.layout_frame: tk.Frame | None = None
        self.dpad_hint_label: tk.Label | None = None
        self.header_frame: tk.Frame | None = None
        self.header_title_label: tk.Label | None = None
        self.header_badge_frame: tk.Frame | None = None
        self.runtime_badges: dict[str, tk.Label] = {}
        self.metric_value_labels: dict[str, tk.Label] = {}
        self.drag_section_name: str | None = None
        self.drag_start_xy: tuple[int, int] | None = None
        self.drag_active = False
        self.drag_target_slot: str | None = None
        self.drag_target_section: str | None = None

        self.layout_orders = {
            "wide": [SECTION_SERIAL, SECTION_STATUS, SECTION_MOTION, SECTION_ADVANCED, SECTION_LOG],
            "compact": [SECTION_SERIAL, SECTION_STATUS, SECTION_MOTION, SECTION_ADVANCED, SECTION_LOG],
            "single": [SECTION_LOG, SECTION_SERIAL, SECTION_MOTION, SECTION_STATUS, SECTION_ADVANCED],
        }

        self._build_ui()
        self._bind_variables()
        self._bind_shortcuts()
        self._refresh_ports()
        self._update_preview()
        self._refresh_runtime_indicators()
        self._poll_log()

    def _build_ui(self):
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        self._build_header()
        self._build_scrollable_body()

        serial_body = self._register_section(SECTION_SERIAL, "串口连接与发送", priority=20)
        self._build_connection(serial_body)
        self._build_send_panel(serial_body)

        motion_body = self._register_section(SECTION_MOTION, "运动控制", priority=30)
        self._build_mode_panel(motion_body)
        self._build_speed_panel(motion_body)
        self._build_dpad(motion_body)

        status_body = self._register_section(SECTION_STATUS, "运行状态", priority=40)
        self._build_status(status_body)
        self._build_dashboard(status_body)

        advanced_body = self._register_section(
            SECTION_ADVANCED,
            "高级诊断",
            priority=50,
            collapsible=True,
        )
        self._build_frame_preview(advanced_body)
        self._build_rx_preview(advanced_body)
        self._build_imu_panel(advanced_body)

        log_body = self._register_section(SECTION_LOG, "通信日志", priority=10)
        self._build_log(log_body)

        self.root.bind("<Configure>", self._on_root_configure)
        self.root.bind_all("<MouseWheel>", self._on_mousewheel)
        self.root.after_idle(lambda: self._apply_responsive_layout(self.root.winfo_width()))

    def _build_header(self):
        header = tk.Frame(self.root, bg=PANEL)
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(1, weight=1)
        self.header_frame = header

        dot = tk.Canvas(header, width=10, height=10, bg=PANEL, highlightthickness=0)
        dot.grid(row=0, column=0, padx=(18, 6), pady=14)
        dot.create_oval(1, 1, 9, 9, fill=ACCENT, outline="")

        self.header_title_label = tk.Label(
            header,
            text="WHEELTEC 底盘串口控制台",
            bg=PANEL,
            fg=TEXT_PRI,
            font=("Microsoft YaHei UI", 13, "bold"),
        )
        self.header_title_label.grid(row=0, column=1, sticky="w", pady=12)

        badge_frame = tk.Frame(header, bg=PANEL)
        badge_frame.grid(row=0, column=2, sticky="e", padx=18, pady=10)
        self.header_badge_frame = badge_frame
        self.status_lbl = self._create_badge(
            badge_frame,
            "未连接",
            bg="#351c1c",
            fg=DANGER,
            font=("Consolas", 10, "bold"),
            pad_x=12,
            pad_y=5,
        )
        self.status_lbl.pack(side="left", padx=(0, 8))
        self.runtime_badges["auto"] = self._create_badge(badge_frame, "连发关", bg="#222831", fg=TEXT_SEC)
        self.runtime_badges["auto"].pack(side="left", padx=4)
        self.runtime_badges["record"] = self._create_badge(badge_frame, "记录关", bg="#222831", fg=TEXT_SEC)
        self.runtime_badges["record"].pack(side="left", padx=4)
        self.runtime_badges["link"] = self._create_badge(badge_frame, "等待上行", bg="#222831", fg=TEXT_SEC)
        self.runtime_badges["link"].pack(side="left", padx=(4, 0))

        ttk.Separator(header, orient="horizontal").grid(row=1, column=0, columnspan=3, sticky="ew")

    def _build_scrollable_body(self):
        body = tk.Frame(self.root, bg=BG)
        body.grid(row=1, column=0, sticky="nsew")
        body.columnconfigure(0, weight=1)
        body.rowconfigure(0, weight=1)

        self.body_scroll_canvas = tk.Canvas(body, bg=BG, highlightthickness=0, bd=0)
        scrollbar = ttk.Scrollbar(body, orient="vertical", command=self.body_scroll_canvas.yview)
        self.body_scroll_canvas.configure(yscrollcommand=scrollbar.set)

        self.body_scroll_canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")

        self.body_scroll_frame = tk.Frame(self.body_scroll_canvas, bg=BG)
        self.body_scroll_window = self.body_scroll_canvas.create_window(
            (0, 0),
            window=self.body_scroll_frame,
            anchor="nw",
        )

        self.body_scroll_frame.columnconfigure(0, weight=1)
        self.body_scroll_frame.bind("<Configure>", self._handle_body_frame_configure)
        self.body_scroll_canvas.bind("<Configure>", self._handle_body_canvas_configure)

        self.layout_frame = tk.Frame(self.body_scroll_frame, bg=BG)
        self.layout_frame.grid(row=0, column=0, sticky="ew", padx=12, pady=(0, 12))

    def _handle_body_frame_configure(self, _event: tk.Event | None):
        if self.body_scroll_canvas is not None:
            self.body_scroll_canvas.configure(scrollregion=self.body_scroll_canvas.bbox("all"))

    def _handle_body_canvas_configure(self, event: tk.Event):
        if self.body_scroll_canvas is None or self.body_scroll_window is None:
            return
        self.body_scroll_canvas.itemconfigure(self.body_scroll_window, width=event.width)
        self._queue_layout_refresh()

    def _on_root_configure(self, event: tk.Event):
        if event.widget is self.root:
            self._queue_layout_refresh()

    def _queue_layout_refresh(self):
        if self.layout_job is not None:
            try:
                self.root.after_cancel(self.layout_job)
            except tk.TclError:
                pass
        self.layout_job = self.root.after(120, lambda: self._apply_responsive_layout(self.root.winfo_width()))

    def _on_mousewheel(self, event: tk.Event):
        if self.body_scroll_canvas is None:
            return
        if self.log_text is not None and self._widget_is_descendant(event.widget, self.log_text):
            return
        if event.delta:
            self.body_scroll_canvas.yview_scroll(int(-event.delta / 120), "units")

    def _widget_is_descendant(self, widget: tk.Misc, ancestor: tk.Misc | None) -> bool:
        if ancestor is None:
            return False

        current: tk.Misc | None = widget
        while current is not None:
            if current == ancestor:
                return True
            parent_name = current.winfo_parent()
            if not parent_name:
                return False
            try:
                current = current.nametowidget(parent_name)
            except KeyError:
                return False
        return False

    def _create_badge(
        self,
        parent: tk.Widget,
        text: str,
        bg: str,
        fg: str,
        font: tuple[str, int] | tuple[str, int, str] = ("Microsoft YaHei UI", 8, "bold"),
        pad_x: int = 10,
        pad_y: int = 4,
    ) -> tk.Label:
        return tk.Label(
            parent,
            text=text,
            bg=bg,
            fg=fg,
            font=font,
            padx=pad_x,
            pady=pad_y,
            bd=0,
        )

    def _set_badge(self, badge: tk.Label | None, text: str, bg: str, fg: str):
        if badge is not None:
            badge.config(text=text, bg=bg, fg=fg)

    def _register_section(
        self,
        name: str,
        title: str,
        priority: int,
        collapsible: bool = False,
    ) -> tk.Frame:
        if self.layout_frame is None:
            raise RuntimeError("layout frame has not been created")

        outer = tk.Frame(self.layout_frame, bg=PANEL, highlightbackground=BORDER, highlightthickness=1)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(2, weight=1)
        self.section_default_border[name] = BORDER

        header = tk.Frame(outer, bg=PANEL)
        header.grid(row=0, column=0, sticky="ew", padx=12, pady=(10, 6))
        header.columnconfigure(0, weight=1)
        header.columnconfigure(1, weight=0)
        self.section_headers[name] = header

        title_label = tk.Label(
            header,
            text=title,
            bg=PANEL,
            fg=TEXT_PRI,
            anchor="w",
            font=("Microsoft YaHei UI", 10, "bold"),
            cursor="fleur",
        )
        title_label.grid(row=0, column=0, sticky="w")

        drag_label = tk.Label(
            header,
            text="拖动",
            bg=PANEL,
            fg=TEXT_DIM,
            font=("Microsoft YaHei UI", 8),
            cursor="fleur",
        )
        drag_column = 2 if collapsible else 1
        drag_label.grid(row=0, column=drag_column, sticky="e", padx=(8, 0))
        self.section_drag_labels[name] = drag_label

        if collapsible:
            toggle_btn = ttk.Button(
                header,
                text="收起",
                style="Ghost.TButton",
                width=6,
                command=lambda section=name: self._toggle_section(section),
            )
            toggle_btn.grid(row=0, column=1, sticky="e", padx=(8, 0))
            self.section_toggles[name] = toggle_btn

        ttk.Separator(outer, orient="horizontal").grid(row=1, column=0, sticky="ew", padx=10)

        body = tk.Frame(outer, bg=PANEL)
        body.grid(row=2, column=0, sticky="nsew", padx=10, pady=8)
        body.columnconfigure(0, weight=1)

        self.section_frames[name] = outer
        self.section_bodies[name] = body
        self.section_order[name] = priority
        self.section_titles[name] = title_label
        self.collapsed_sections[name] = False
        self._bind_section_drag(name, header, title_label, drag_label)
        return body

    def _bind_section_drag(self, name: str, *widgets: tk.Widget):
        for widget in widgets:
            widget.bind("<ButtonPress-1>", lambda event, section=name: self._start_section_drag(section, event))
            widget.bind("<B1-Motion>", self._on_section_drag_motion)
            widget.bind("<ButtonRelease-1>", self._finish_section_drag)

    def _start_section_drag(self, name: str, event: tk.Event):
        self.drag_section_name = name
        self.drag_start_xy = (event.x_root, event.y_root)
        self.drag_active = False
        self.drag_target_slot = None
        self.drag_target_section = None
        frame = self.section_frames.get(name)
        if frame is not None:
            frame.configure(highlightbackground=ACCENT2, highlightthickness=2)
        if self.layout_mode != "single":
            self._show_docking_guides()

    def _on_section_drag_motion(self, event: tk.Event):
        if self.drag_section_name is None or self.drag_start_xy is None:
            return

        if not self.drag_active:
            start_x, start_y = self.drag_start_xy
            if abs(event.x_root - start_x) + abs(event.y_root - start_y) < 8:
                return
            self.drag_active = True

        if self.layout_mode == "single":
            section_name = self._find_section_at_point(event.x_root, event.y_root)
            if section_name == self.drag_section_name:
                section_name = None
            slot_name = None
            if section_name is not None:
                frame = self.section_frames.get(section_name)
                if frame is not None:
                    midpoint = frame.winfo_rooty() + frame.winfo_height() / 2
                    slot_name = "after" if event.y_root > midpoint else "before"
            self._set_drag_target(slot_name=slot_name, section_name=section_name)
        else:
            self._update_docking_guides()
            slot_name = self._find_desktop_drop_slot(event.x_root, event.y_root)
            if slot_name is not None:
                slot_section = self._get_desktop_slot_map().get(slot_name)
                if slot_section == self.drag_section_name:
                    slot_name = None
                    slot_section = None
            else:
                slot_section = None
            self._set_drag_target(slot_name=slot_name, section_name=slot_section)

    def _finish_section_drag(self, event: tk.Event):
        dragged = self.drag_section_name
        target_slot = self.drag_target_slot
        target_section = self.drag_target_section

        self._clear_drag_visuals()
        if dragged is not None and self.drag_active:
            if self.layout_mode == "single" and target_section is not None:
                self._insert_section_relative(dragged, target_section, target_slot == "after")
            elif self.layout_mode != "single" and target_slot is not None:
                self._move_section_to_slot(dragged, target_slot)
            self._queue_layout_refresh()

        self.drag_section_name = None
        self.drag_start_xy = None
        self.drag_active = False
        self.drag_target_slot = None
        self.drag_target_section = None
        self._hide_docking_guides()

    def _find_section_at_point(self, x_root: int, y_root: int) -> str | None:
        for name, frame in self.section_frames.items():
            if not frame.winfo_ismapped():
                continue
            left = frame.winfo_rootx()
            top = frame.winfo_rooty()
            right = left + frame.winfo_width()
            bottom = top + frame.winfo_height()
            if left <= x_root <= right and top <= y_root <= bottom:
                return name
        return None

    def _set_drag_target(self, slot_name: str | None = None, section_name: str | None = None):
        if self.drag_target_section == section_name and self.drag_target_slot == slot_name:
            return
        if self.drag_target_section is not None:
            self._restore_section_border(self.drag_target_section)
        self.drag_target_slot = slot_name
        self.drag_target_section = section_name
        if section_name is not None:
            frame = self.section_frames.get(section_name)
            if frame is not None:
                frame.configure(highlightbackground=ACCENT, highlightthickness=2)
        self._highlight_docking_guide(slot_name)

    def _restore_section_border(self, name: str):
        frame = self.section_frames.get(name)
        if frame is not None:
            frame.configure(
                highlightbackground=self.section_default_border.get(name, BORDER),
                highlightthickness=1,
            )

    def _clear_drag_visuals(self):
        if self.drag_section_name is not None:
            self._restore_section_border(self.drag_section_name)
        if self.drag_target_section is not None:
            self._restore_section_border(self.drag_target_section)
        self._highlight_docking_guide(None)

    def _get_desktop_slot_map(self) -> dict[str, str]:
        order = self.layout_orders["compact" if self.layout_mode == "compact" else "wide"]
        return dict(zip(DESKTOP_SLOT_NAMES, order))

    def _show_docking_guides(self):
        if self.layout_frame is None:
            return
        if not self.docking_guides:
            title_map = {
                "top_left": "左上",
                "top_right": "右上",
                "bottom_left": "左下",
                "bottom_right": "右下",
                "bottom_full": "底部",
            }
            for slot_name, title in title_map.items():
                guide = tk.Label(
                    self.layout_frame,
                    text=title,
                    bg="#152433",
                    fg=TEXT_PRI,
                    font=("Microsoft YaHei UI", 9, "bold"),
                    bd=0,
                    padx=10,
                    pady=8,
                )
                self.docking_guides[slot_name] = guide
        self._update_docking_guides()
        for guide in self.docking_guides.values():
            guide.lift()

    def _hide_docking_guides(self):
        for guide in self.docking_guides.values():
            guide.place_forget()

    def _update_docking_guides(self):
        if self.layout_mode == "single":
            self._hide_docking_guides()
            return
        if self.layout_frame is None:
            return

        self.layout_frame.update_idletasks()
        slot_map = self._get_desktop_slot_map()
        for slot_name, section_name in slot_map.items():
            guide = self.docking_guides.get(slot_name)
            frame = self.section_frames.get(section_name)
            if guide is None or frame is None or not frame.winfo_ismapped():
                continue
            width = max(92, min(frame.winfo_width() - 28, 120))
            height = 42
            x = frame.winfo_x() + max(12, (frame.winfo_width() - width) // 2)
            y = frame.winfo_y() + max(12, min(frame.winfo_height() - height - 12, 20))
            guide.place(x=x, y=y, width=width, height=height)

    def _highlight_docking_guide(self, slot_name: str | None):
        for name, guide in self.docking_guides.items():
            if slot_name == name:
                guide.config(bg="#1f6feb", fg="white")
            else:
                guide.config(bg="#152433", fg=TEXT_PRI)

    def _find_desktop_drop_slot(self, x_root: int, y_root: int) -> str | None:
        for slot_name, guide in self.docking_guides.items():
            if not guide.winfo_ismapped():
                continue
            left = guide.winfo_rootx()
            top = guide.winfo_rooty()
            right = left + guide.winfo_width()
            bottom = top + guide.winfo_height()
            if left <= x_root <= right and top <= y_root <= bottom:
                return slot_name
        return None

    def _insert_section_relative(self, dragged: str, target: str, after: bool):
        order = list(self.layout_orders["single"])
        if dragged not in order or target not in order:
            return
        order.remove(dragged)
        target_index = order.index(target)
        if after:
            target_index += 1
        order.insert(target_index, dragged)
        self.layout_orders["single"] = order

    def _move_section_to_slot(self, dragged: str, slot_name: str):
        if slot_name not in DESKTOP_SLOT_NAMES:
            return
        target_index = DESKTOP_SLOT_NAMES.index(slot_name)
        for mode in ("wide", "compact"):
            order = list(self.layout_orders[mode])
            if dragged not in order:
                continue
            order.remove(dragged)
            order.insert(target_index, dragged)
            self.layout_orders[mode] = order

    def _toggle_section(self, name: str):
        self._set_section_collapsed(name, not self.collapsed_sections.get(name, False))
        self._queue_layout_refresh()

    def _set_section_collapsed(self, name: str, collapsed: bool):
        body = self.section_bodies.get(name)
        if body is None:
            return
        if collapsed:
            body.grid_remove()
        else:
            body.grid()
        self.collapsed_sections[name] = collapsed
        toggle = self.section_toggles.get(name)
        if toggle is not None:
            toggle.config(text="展开" if collapsed else "收起")

    def _apply_responsive_layout(self, width: int):
        self.layout_job = None
        if self.layout_frame is None:
            return

        previous_mode = self.layout_mode
        if width >= LAYOUT_WIDE_MIN:
            layout_mode = "wide"
        elif width >= LAYOUT_COMPACT_MIN:
            layout_mode = "compact"
        else:
            layout_mode = "single"

        self.layout_mode = layout_mode

        for section in self.section_frames.values():
            section.grid_forget()

        for column in range(3):
            self.layout_frame.columnconfigure(column, weight=0, uniform="")

        if layout_mode == "single":
            grid_pad_x = 4
            grid_pad_y = 4
            self.layout_frame.columnconfigure(0, weight=1)
            self.layout_frame.columnconfigure(0, minsize=0)
            order = self.layout_orders["single"]
            for row, name in enumerate(order):
                self.section_frames[name].grid(
                    row=row,
                    column=0,
                    columnspan=1,
                    sticky="ew",
                    padx=grid_pad_x,
                    pady=grid_pad_y,
                )
            if previous_mode != "single":
                self._set_section_collapsed(SECTION_ADVANCED, True)
        else:
            grid_pad_x = 6 if layout_mode == "compact" else 8
            grid_pad_y = 6 if layout_mode == "compact" else 8
            if layout_mode == "compact":
                self.layout_frame.columnconfigure(0, weight=5, minsize=500)
                self.layout_frame.columnconfigure(1, weight=6, minsize=560)
            else:
                self.layout_frame.columnconfigure(0, weight=4, minsize=480)
                self.layout_frame.columnconfigure(1, weight=7, minsize=680)
            order = self.layout_orders["compact" if layout_mode == "compact" else "wide"]
            slot_specs = (
                {"row": 0, "column": 0, "columnspan": 1},
                {"row": 0, "column": 1, "columnspan": 1},
                {"row": 1, "column": 0, "columnspan": 1},
                {"row": 1, "column": 1, "columnspan": 1},
                {"row": 2, "column": 0, "columnspan": 2},
            )
            for name, slot in zip(order, slot_specs):
                self.section_frames[name].grid(
                    row=slot["row"],
                    column=slot["column"],
                    columnspan=slot["columnspan"],
                    sticky="ew",
                    padx=grid_pad_x,
                    pady=grid_pad_y,
                )
            if previous_mode == "single" or self.collapsed_sections.get(SECTION_ADVANCED, False):
                self._set_section_collapsed(SECTION_ADVANCED, False)

        self._update_visual_density(layout_mode, width)
        self._update_wraplengths(layout_mode)
        self._update_frame_bar_layouts(layout_mode)
        if self.drag_active and layout_mode != "single":
            self._update_docking_guides()
        else:
            self._hide_docking_guides()
        self._handle_body_frame_configure(None)

    def _update_visual_density(self, layout_mode: str, width: int):
        if self.header_title_label is not None:
            self.header_title_label.config(
                font=("Microsoft YaHei UI", 11 if layout_mode == "single" else 13, "bold"),
            )
        if self.status_lbl is not None:
            self.status_lbl.config(
                font=("Consolas", 9 if layout_mode == "single" else 10, "bold"),
                padx=10 if layout_mode == "single" else 12,
                pady=4 if layout_mode == "single" else 5,
            )

        title_font_size = 9 if layout_mode == "single" else 10
        for label in self.section_titles.values():
            label.config(font=("Microsoft YaHei UI", title_font_size, "bold"))
        for label in self.section_drag_labels.values():
            label.config(
                font=("Microsoft YaHei UI", 7 if layout_mode == "single" else 8),
                fg=TEXT_SEC if not self.drag_active else ACCENT,
            )
        for guide in self.docking_guides.values():
            guide.config(font=("Microsoft YaHei UI", 8 if layout_mode == "single" else 9, "bold"))

        if hasattr(self, "log_text"):
            self.log_text.config(height=10 if layout_mode == "single" else 12 if layout_mode == "compact" else 14)
        if hasattr(self, "imu_text"):
            self.imu_text.config(height=8 if layout_mode == "single" else 9 if layout_mode == "compact" else 10)
        for badge_name, badge in self.runtime_badges.items():
            badge.config(
                font=("Microsoft YaHei UI", 7 if layout_mode == "single" else 8, "bold"),
                padx=8 if layout_mode == "single" else 10,
                pady=3 if layout_mode == "single" else 4,
            )
        for label in self.metric_value_labels.values():
            label.config(font=("Consolas", 10 if layout_mode == "single" else 11 if layout_mode == "compact" else 12, "bold"))

    def _update_wraplengths(self, layout_mode: str):
        width = max(self.root.winfo_width(), 360)
        if layout_mode == "single":
            status_wrap = max(220, width - 150)
            hint_wrap = status_wrap
        elif layout_mode == "compact":
            status_wrap = 320
            hint_wrap = 340
        else:
            status_wrap = 420
            hint_wrap = 360

        for label in self.status_value_labels:
            label.config(wraplength=status_wrap)
        if self.dpad_hint_label is not None:
            self.dpad_hint_label.config(wraplength=hint_wrap)

    def _update_frame_bar_layouts(self, layout_mode: str):
        for wrap, columns in self.frame_bar_groups:
            total = len(columns)
            if layout_mode == "single":
                columns_per_row = 4 if total <= 12 else 5
                value_width = 3
                value_font = ("Consolas", 10, "bold")
                caption_font = ("Consolas", 6)
            elif layout_mode == "compact":
                columns_per_row = 6 if total <= 12 else 8
                value_width = 4
                value_font = ("Consolas", 10, "bold")
                caption_font = ("Consolas", 7)
            else:
                columns_per_row = 11 if total <= 12 else 12
                value_width = 4
                value_font = ("Consolas", 11, "bold")
                caption_font = ("Consolas", 7)

            for column in range(12):
                wrap.grid_columnconfigure(column, weight=1 if column < columns_per_row else 0, uniform="framebar")
            for index, item in enumerate(columns):
                children = item.winfo_children()
                if len(children) >= 2:
                    children[0].config(width=value_width, font=value_font)
                    children[1].config(font=caption_font)
                item.grid(
                    row=index // columns_per_row,
                    column=index % columns_per_row,
                    padx=3 if layout_mode == "single" else 4,
                    pady=3 if layout_mode == "single" else 4,
                    sticky="nsew",
                )

    def _refresh_runtime_indicators(self):
        port_name = self.port_var.get().strip() or "串口"
        if self.connected:
            conn_text = port_name if len(port_name) <= 12 else f"{port_name[:12]}..."
            self._set_badge(self.status_lbl, conn_text, "#173126", SUCCESS)
            self._set_badge(self.runtime_badges.get("connection_panel"), "串口在线", "#173126", SUCCESS)
        else:
            self._set_badge(self.status_lbl, "未连接", "#351c1c", DANGER)
            self._set_badge(self.runtime_badges.get("connection_panel"), "串口离线", "#351c1c", DANGER)

        if self.auto_send:
            self._set_badge(self.runtime_badges.get("auto"), "连发开", "#16324a", ACCENT)
            self._set_badge(self.runtime_badges.get("auto_panel"), "自动发送开启", "#16324a", ACCENT)
        else:
            self._set_badge(self.runtime_badges.get("auto"), "连发关", "#222831", TEXT_SEC)
            self._set_badge(self.runtime_badges.get("auto_panel"), "自动发送关闭", "#222831", TEXT_SEC)

        if self.log_writer is not None:
            self._set_badge(self.runtime_badges.get("record"), "记录开", "#3a2412", WARNING)
            self._set_badge(self.runtime_badges.get("record_panel"), "记录开启", "#3a2412", WARNING)
        else:
            self._set_badge(self.runtime_badges.get("record"), "记录关", "#222831", TEXT_SEC)
            self._set_badge(self.runtime_badges.get("record_panel"), "记录关闭", "#222831", TEXT_SEC)

        if self.connected and self.rx_count > 0:
            self._set_badge(self.runtime_badges.get("link"), "上行正常", "#173126", SUCCESS)
        elif self.connected:
            self._set_badge(self.runtime_badges.get("link"), "等待上行", "#3a2412", WARNING)
        else:
            self._set_badge(self.runtime_badges.get("link"), "链路离线", "#222831", TEXT_SEC)

    def _build_connection(self, parent: tk.Widget):
        card = self._card(parent, "串口配置")
        card.columnconfigure(1, weight=1)
        card.columnconfigure(3, weight=1)

        tk.Label(card, text="端口", anchor="w", bg=PANEL, fg=TEXT_SEC).grid(row=0, column=0, sticky="w")
        self.port_cb = ttk.Combobox(card, textvariable=self.port_var, state="readonly")
        self.port_cb.grid(row=0, column=1, sticky="ew", padx=(8, 8), pady=(0, 8))
        ttk.Button(card, text="刷新", style="Secondary.TButton", command=self._refresh_ports).grid(
            row=0,
            column=2,
            sticky="ew",
            pady=(0, 8),
        )

        tk.Label(card, text="波特率", anchor="w", bg=PANEL, fg=TEXT_SEC).grid(row=1, column=0, sticky="w")
        self.baud_cb = ttk.Combobox(
            card,
            textvariable=self.baud_var,
            state="readonly",
            values=["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"],
        )
        self.baud_cb.grid(row=1, column=1, sticky="ew", padx=(8, 8))

        tk.Label(card, text="超时(s)", anchor="w", bg=PANEL, fg=TEXT_SEC).grid(row=1, column=2, sticky="w")
        timeout_entry = ttk.Entry(card, textvariable=self.timeout_var, style="Dark.TEntry")
        timeout_entry.grid(row=1, column=3, sticky="ew")

        self.conn_btn = ttk.Button(
            card,
            text="连接串口",
            style="Primary.TButton",
            command=self._toggle_connection,
        )
        self.conn_btn.grid(row=2, column=0, columnspan=4, sticky="ew", pady=(10, 0))

    def _build_status(self, parent: tk.Widget):
        card = self._card(parent, "运行摘要")

        metrics = tk.Frame(card, bg=PANEL)
        metrics.pack(fill="x", pady=(0, 10))
        for column in range(2):
            metrics.grid_columnconfigure(column, weight=1, uniform="metric")

        metric_specs = (
            ("tx", "发送帧数", self.tx_count_var, ACCENT),
            ("rx", "接收帧数", self.rx_count_var, SUCCESS),
            ("send", "最近发送", self.last_send_var, ACCENT2),
            ("battery", "电池状态", self.battery_var, WARNING),
        )
        for index, (key, title, variable, color) in enumerate(metric_specs):
            panel = tk.Frame(metrics, bg="#0f141b", highlightbackground=BORDER, highlightthickness=1)
            panel.grid(row=index // 2, column=index % 2, sticky="nsew", padx=4, pady=4)
            tk.Label(
                panel,
                text=title,
                bg="#0f141b",
                fg=TEXT_DIM,
                anchor="w",
                font=("Microsoft YaHei UI", 8),
            ).pack(fill="x", padx=10, pady=(8, 2))
            value_label = tk.Label(
                panel,
                textvariable=variable,
                bg="#0f141b",
                fg=color,
                anchor="w",
                font=("Consolas", 12, "bold"),
            )
            value_label.pack(fill="x", padx=10, pady=(0, 8))
            self.metric_value_labels[key] = value_label

        rows = (
            ("接收状态", self.rx_state_var),
            ("最近上行", self.rx_value_var),
        )
        for title, variable in rows:
            row = tk.Frame(card, bg=PANEL)
            row.pack(fill="x", pady=2)
            row.columnconfigure(1, weight=1)
            tk.Label(row, text=title, anchor="w", bg=PANEL, fg=TEXT_SEC).grid(row=0, column=0, sticky="w")
            value_label = tk.Label(
                row,
                textvariable=variable,
                anchor="w",
                justify="left",
                bg=PANEL,
                fg=TEXT_PRI if title != "接收状态" else WARNING,
                font=("Consolas", 9),
            )
            value_label.grid(row=0, column=1, sticky="ew", padx=(12, 0))
            self.status_value_labels.append(value_label)

    def _build_speed_panel(self, parent: tk.Widget):
        card = self._card(parent, "速度设置")

        for axis_name, label, unit, low, high, resolution in self.AXIS_CONFIG:
            axis_var = getattr(self, f"{axis_name}_var")
            entry_var = tk.StringVar(value=str(axis_var.get()))
            self.axis_entry_vars[axis_name] = entry_var
            row = tk.Frame(card, bg=PANEL)
            row.pack(fill="x", pady=(0, 2))
            row.columnconfigure(1, weight=1)

            tk.Label(row, text=label, anchor="w", bg=PANEL, fg=TEXT_SEC).grid(row=0, column=0, sticky="w")

            entry = ttk.Entry(
                row,
                width=8,
                justify="center",
                style="Dark.TEntry",
                textvariable=entry_var,
            )
            entry.grid(row=0, column=1, sticky="e", padx=(8, 8))
            entry.bind("<Return>", lambda _e, n=axis_name: self._apply_entry_value(n))
            entry.bind("<FocusOut>", lambda _e, n=axis_name: self._apply_entry_value(n))
            self.speed_entries[axis_name] = entry

            tk.Label(row, text=unit, anchor="e", bg=PANEL, fg=TEXT_DIM).grid(row=0, column=2, sticky="e")
            value_label = tk.Label(
                row,
                text="0",
                width=6,
                anchor="e",
                bg=PANEL,
                fg=ACCENT,
                font=("Consolas", 10, "bold"),
            )
            value_label.grid(row=0, column=3, sticky="e", padx=(12, 0))
            self.speed_labels[axis_name] = value_label

            scale = tk.Scale(
                card,
                from_=low,
                to=high,
                resolution=resolution,
                orient="horizontal",
                variable=axis_var,
                bg=PANEL,
                fg=TEXT_SEC,
                troughcolor=BORDER,
                activebackground=ACCENT,
                highlightthickness=0,
                bd=0,
                showvalue=False,
            )
            scale.pack(fill="x", pady=(0, 6))
            self.speed_scales[axis_name] = scale

        step_row = tk.Frame(card, bg=PANEL)
        step_row.pack(fill="x", pady=(8, 0))
        tk.Label(step_row, text="控制步进", anchor="w", bg=PANEL, fg=TEXT_SEC).pack(side="left")
        for step in (50, 100, 200, 500, 1000):
            ttk.Button(
                step_row,
                text=str(step),
                style="Secondary.TButton",
                width=4,
                command=lambda value=step: self.step_var.set(value),
            ).pack(side="left", padx=(8 if step == 50 else 4, 0))
        tk.Label(step_row, textvariable=self.step_var, bg=PANEL, fg=ACCENT2, font=("Consolas", 10, "bold")).pack(side="left", padx=(10, 0))

        ttk.Button(card, text="全部清零", style="Secondary.TButton", command=self._zero_speeds).pack(
            fill="x",
            pady=(10, 0),
        )

    def _build_mode_panel(self, parent: tk.Widget):
        card = self._card(parent, "控制模式")
        card.columnconfigure(1, weight=1)

        tk.Label(card, text="Mode", anchor="w", bg=PANEL, fg=TEXT_SEC).grid(row=0, column=0, sticky="w")
        self.mode_cb = ttk.Combobox(
            card,
            textvariable=self.mode_var,
            state="readonly",
            values=list(CONTROL_MODES.keys()),
        )
        self.mode_cb.grid(row=0, column=1, sticky="ew", padx=(8, 0), pady=(0, 8))
        self.mode_cb.current(0)

        ttk.Checkbutton(
            card,
            text="断线自动重连",
            variable=self.auto_reconnect_var,
            style="Dark.TCheckbutton",
        ).grid(row=1, column=0, columnspan=2, sticky="w")

    def _build_dpad(self, parent: tk.Widget):
        card = self._card(parent, "方向控制 (WASD / QE)")

        grid = tk.Frame(card, bg=PANEL)
        grid.pack()
        for column in range(4):
            grid.grid_columnconfigure(column, weight=1)

        pad_cfg = (
            ("↖", 0, 0, lambda: self._set_vel(1, 1, 0), "Pad.TButton"),
            ("↑", 0, 1, lambda: self._set_vel(1, 0, 0), "Pad.TButton"),
            ("↗", 0, 2, lambda: self._set_vel(1, -1, 0), "Pad.TButton"),
            ("⟲", 0, 3, lambda: self._set_vel(0, 0, 1), "Pad.TButton"),
            ("←", 1, 0, lambda: self._set_vel(0, 1, 0), "Pad.TButton"),
            ("■", 1, 1, self._emergency_stop, "PadDanger.TButton"),
            ("→", 1, 2, lambda: self._set_vel(0, -1, 0), "Pad.TButton"),
            ("⟳", 1, 3, lambda: self._set_vel(0, 0, -1), "Pad.TButton"),
            ("↙", 2, 0, lambda: self._set_vel(-1, 1, 0), "Pad.TButton"),
            ("↓", 2, 1, lambda: self._set_vel(-1, 0, 0), "Pad.TButton"),
            ("↘", 2, 2, lambda: self._set_vel(-1, -1, 0), "Pad.TButton"),
        )

        self.pointer_pad_map = {
            "↖": (1, 1, 0),
            "↑": (1, 0, 0),
            "↗": (1, -1, 0),
            "⟲": (0, 0, 1),
            "←": (0, 1, 0),
            "→": (0, -1, 0),
            "⟳": (0, 0, -1),
            "↙": (-1, 1, 0),
            "↓": (-1, 0, 0),
            "↘": (-1, -1, 0),
        }

        for text, row, column, command, style in pad_cfg:
            button = ttk.Button(
                grid,
                text=text,
                width=3,
                style=style,
                command=command if text == "■" else None,
            )
            button.grid(
                row=row,
                column=column,
                padx=4,
                pady=4,
                sticky="nsew",
            )
            if text in self.pointer_pad_map:
                button.bind("<ButtonPress-1>", lambda _e, key=text: self._start_pointer_motion(key))
                button.bind("<ButtonRelease-1>", lambda _e, key=text: self._stop_pointer_motion(key))
                button.bind("<Leave>", lambda _e, key=text: self._stop_pointer_motion(key))

        self.dpad_hint_label = tk.Label(
            card,
            text="键盘按住 WASD / QE 持续运动；鼠标按住方向键运动、松手自动归零。回车发送一次，空格急停。",
            bg=PANEL,
            fg=TEXT_DIM,
            font=("Microsoft YaHei UI", 8),
            justify="left",
        )
        self.dpad_hint_label.pack(pady=(6, 0), anchor="w")

    def _build_send_panel(self, parent: tk.Widget):
        card = self._card(parent, "发送控制")
        card.columnconfigure(1, weight=1)
        card.columnconfigure(2, weight=1)
        card.columnconfigure(3, weight=1)

        tk.Label(card, text="周期(s)", anchor="w", bg=PANEL, fg=TEXT_SEC).grid(row=0, column=0, sticky="w")
        interval_entry = ttk.Entry(card, textvariable=self.auto_interval_var, style="Dark.TEntry")
        interval_entry.grid(row=0, column=1, sticky="ew", padx=(8, 0), pady=(0, 8))

        tk.Label(card, text="防抖(ms)", anchor="w", bg=PANEL, fg=TEXT_SEC).grid(row=0, column=2, sticky="w", padx=(12, 0))
        debounce_entry = ttk.Entry(card, textvariable=self.auto_send_delay_var, style="Dark.TEntry", width=6)
        debounce_entry.grid(row=0, column=3, sticky="ew", pady=(0, 8))

        ttk.Checkbutton(
            card,
            text="速度变化自动发送",
            variable=self.auto_send_on_change_var,
            style="Dark.TCheckbutton",
        ).grid(row=1, column=0, columnspan=2, sticky="w")
        tk.Label(card, textvariable=self.auto_send_delay_var, bg=PANEL, fg=TEXT_DIM, font=("Consolas", 8, "bold")).grid(
            row=1,
            column=2,
            sticky="e",
            padx=(12, 0),
        )
        tk.Label(card, text="ms", bg=PANEL, fg=TEXT_DIM, font=("Microsoft YaHei UI", 8)).grid(row=1, column=3, sticky="w", padx=(4, 0))

        btn_row = tk.Frame(card, bg=PANEL)
        btn_row.grid(row=2, column=0, columnspan=4, sticky="ew", pady=(10, 0))
        for column in range(3):
            btn_row.grid_columnconfigure(column, weight=1)

        ttk.Button(btn_row, text="发送一次", style="Primary.TButton", command=self._send_once).grid(
            row=0,
            column=0,
            sticky="ew",
            padx=(0, 4),
        )

        self.auto_btn = ttk.Button(
            btn_row,
            text="连续发送",
            style="Secondary.TButton",
            command=self._toggle_auto,
        )
        self.auto_btn.grid(row=0, column=1, sticky="ew", padx=4)

        ttk.Button(btn_row, text="急停发送", style="Danger.TButton", command=self._emergency_stop).grid(
            row=0,
            column=2,
            sticky="ew",
            padx=(4, 0),
        )

    def _build_dashboard(self, parent: tk.Widget):
        card = self._card(parent, "实时状态")

        badge_row = tk.Frame(card, bg=PANEL)
        badge_row.pack(fill="x", pady=(0, 10))
        self.runtime_badges["connection_panel"] = self._create_badge(badge_row, "串口离线", bg="#351c1c", fg=DANGER)
        self.runtime_badges["connection_panel"].pack(side="left", padx=(0, 6))
        self.runtime_badges["auto_panel"] = self._create_badge(badge_row, "自动发送关闭", bg="#222831", fg=TEXT_SEC)
        self.runtime_badges["auto_panel"].pack(side="left", padx=6)
        self.runtime_badges["record_panel"] = self._create_badge(badge_row, "记录关闭", bg="#222831", fg=TEXT_SEC)
        self.runtime_badges["record_panel"].pack(side="left", padx=6)
        tk.Label(badge_row, text="电量按", bg=PANEL, fg=TEXT_DIM, font=("Microsoft YaHei UI", 8)).pack(side="right")
        battery_cells_cb = ttk.Combobox(
            badge_row,
            textvariable=self.battery_cells_var,
            state="readonly",
            values=["3S", "4S", "6S"],
            width=4,
        )
        battery_cells_cb.pack(side="right", padx=(6, 4))

        bat_row = tk.Frame(card, bg=PANEL)
        bat_row.pack(fill="x")
        bat_row.columnconfigure(1, weight=1)

        tk.Label(bat_row, text="电池", anchor="w", bg=PANEL, fg=TEXT_SEC).grid(row=0, column=0, sticky="w")
        self.battery_bar = ttk.Progressbar(
            bat_row,
            mode="determinate",
            style="Battery.Horizontal.TProgressbar",
            variable=tk.DoubleVar(value=0),
        )
        self.battery_bar.grid(row=0, column=1, sticky="ew", padx=(10, 10))
        tk.Label(
            bat_row,
            textvariable=self.battery_var,
            width=8,
            anchor="e",
            bg=PANEL,
            fg=SUCCESS,
            font=("Consolas", 10, "bold"),
        ).grid(row=0, column=2, sticky="e")
        tk.Label(
            bat_row,
            textvariable=self.battery_pct_var,
            width=5,
            anchor="e",
            bg=PANEL,
            fg=TEXT_DIM,
            font=("Consolas", 9),
        ).grid(row=0, column=3, sticky="e", padx=(8, 0))

    def _build_imu_panel(self, parent: tk.Widget):
        card = self._card(parent, "详细 IMU / 速度")
        imu_card = tk.Frame(card, bg="#0a0e13", highlightbackground=BORDER, highlightthickness=1)
        imu_card.pack(fill="x")

        self.imu_text = tk.Text(
            imu_card,
            height=10,
            bg="#0a0e13",
            fg=ACCENT,
            font=("Consolas", 9),
            relief="flat",
            bd=0,
            insertbackground=TEXT_PRI,
            state="disabled",
        )
        self.imu_text.pack(fill="x", padx=6, pady=6)

    def _build_frame_preview(self, parent: tk.Widget):
        card = self._card(parent, "下行数据帧预览")
        self.byte_labels = self._build_frame_bar(
            card,
            ("帧头", "Mode", "预留", "Vx_H", "Vx_L", "Vy_H", "Vy_L", "Vz_H", "Vz_L", "BCC", "帧尾"),
        )

    def _build_rx_preview(self, parent: tk.Widget):
        card = self._card(parent, "最近上行数据帧")

        rx_fields = (
            "HEAD", "Flag", "Vx_H", "Vx_L", "Vy_H", "Vy_L", "Vz_H", "Vz_L",
            "Ax_H", "Ax_L", "Ay_H", "Ay_L", "Az_H", "Az_L",
            "Gx_H", "Gx_L", "Gy_H", "Gy_L", "Gz_H", "Gz_L",
            "Pwr_H", "Pwr_L", "BCC", "TAIL",
        )
        self.last_rx_byte_labels = self._build_frame_bar(card, rx_fields)
        self._set_frame_labels(self.last_rx_byte_labels, bytes(RX_FRAME_LEN))

    def _build_frame_bar(self, parent: tk.Widget, fields: tuple[str, ...]) -> list[tk.Label]:
        wrap = tk.Frame(parent, bg="#0a0e13", highlightbackground=BORDER, highlightthickness=1)
        wrap.pack(fill="x", padx=4, pady=4)

        labels: list[tk.Label] = []
        columns: list[tk.Frame] = []
        for field in fields:
            col = tk.Frame(wrap, bg="#0a0e13")
            value = tk.Label(
                col,
                text="00",
                width=4,
                bg=BTN_BG,
                fg=TEXT_SEC,
                font=("Consolas", 11, "bold"),
            )
            value.pack()
            tk.Label(col, text=field, bg="#0a0e13", fg=TEXT_DIM, font=("Consolas", 7)).pack()
            labels.append(value)
            columns.append(col)

        self.frame_bar_groups.append((wrap, columns))
        return labels

    def _build_log(self, parent: tk.Widget):
        card = self._card(parent, "日志窗口", expand=True)

        toolbar = tk.Frame(card, bg=PANEL)
        toolbar.pack(fill="x", pady=(0, 6))
        self.log_btn = ttk.Button(
            toolbar,
            text="开始记录",
            style="Secondary.TButton",
            command=self._toggle_logging,
        )
        self.log_btn.pack(side="left")
        ttk.Button(toolbar, text="清空日志", style="Secondary.TButton", command=self._clear_log).pack(side="right")

        self.log_text = scrolledtext.ScrolledText(
            card,
            height=14,
            bg="#0a0e13",
            fg=TEXT_SEC,
            font=("Consolas", 9),
            relief="flat",
            bd=0,
            wrap="word",
            insertbackground=TEXT_PRI,
            selectbackground=ACCENT,
            state="disabled",
        )
        self.log_text.pack(fill="both", expand=True)
        self.log_text.tag_config("tx", foreground=ACCENT)
        self.log_text.tag_config("rx", foreground=SUCCESS)
        self.log_text.tag_config("warn", foreground=WARNING)
        self.log_text.tag_config("err", foreground=DANGER)
        self.log_text.tag_config("inf", foreground=TEXT_PRI)
        self.log_text.tag_config("ack_ok", foreground=SUCCESS)
        self.log_text.tag_config("ack_fail", foreground=DANGER)
        self.log_text.tag_config("pending", foreground=PENDING)

    def _card(self, parent: tk.Widget, title: str, expand: bool = False) -> tk.Frame:
        outer = tk.Frame(parent, bg="#11161d", highlightbackground=BORDER, highlightthickness=1)
        outer.pack(fill="both" if expand else "x", expand=expand, pady=(0, 8))
        tk.Label(
            outer,
            text=f"  {title}",
            bg="#11161d",
            fg=TEXT_SEC,
            anchor="w",
            font=("Microsoft YaHei UI", 8, "bold"),
        ).pack(fill="x", pady=(6, 2))
        ttk.Separator(outer, orient="horizontal").pack(fill="x")
        inner = tk.Frame(outer, bg=PANEL)
        inner.pack(fill="both", expand=True, padx=10, pady=8)
        return inner

    def _bind_variables(self):
        for axis_name, *_rest in self.AXIS_CONFIG:
            axis_var = getattr(self, f"{axis_name}_var")
            axis_var.trace_add("write", lambda *_args, name=axis_name: self._on_axis_change(name))
        self.mode_cb.bind("<<ComboboxSelected>>", self._on_mode_selected)

    def _bind_shortcuts(self):
        # 多键同时支持：记录每个按键的按下状态
        self.key_map = {
            "w": (1, 0, 0), "s": (-1, 0, 0),
            "a": (0, 1, 0), "d": (0, -1, 0),
            "q": (0, 0, 1), "e": (0, 0, -1),
        }
        self.root.bind("<KeyPress>", self._on_key_press)
        self.root.bind("<KeyRelease>", self._on_key_release)
        self.root.bind("<space>", lambda _e: self._emergency_stop())
        self.root.bind("<Return>", lambda _e: self._send_once())
        self.root.bind("<F5>", lambda _e: self._refresh_ports())

    def _on_key_press(self, event: tk.Event):
        key = event.keysym.lower()
        if key in self.key_map:
            self.keys_pressed.add(key)
            self._apply_keyspeed()

    def _on_key_release(self, event: tk.Event):
        key = event.keysym.lower()
        if key in self.key_map:
            self.keys_pressed.discard(key)
            self._apply_keyspeed()

    def _apply_keyspeed(self):
        """根据当前按下的键计算合成速度"""
        step = self.step_var.get()
        vx, vy, vz = 0, 0, 0
        for key in self.keys_pressed:
            dx, dy, dz = self.key_map[key]
            vx += dx * step
            vy += dy * step
            vz += dz * 500
        self.vx_var.set(max(-32767, min(32767, vx)))
        self.vy_var.set(max(-32767, min(32767, vy)))
        self.vz_var.set(max(-32767, min(32767, vz)))

    def _on_axis_change(self, axis_name: str):
        value = getattr(self, f"{axis_name}_var").get()
        self.speed_labels[axis_name].config(text=str(value))
        entry_var = self.axis_entry_vars[axis_name]
        if entry_var.get() != str(value):
            entry_var.set(str(value))
        self._update_preview()
        self._schedule_auto_send(reason=f"{axis_name} changed")

    def _apply_entry_value(self, axis_name: str):
        raw = self.axis_entry_vars[axis_name].get().strip()
        axis_var = getattr(self, f"{axis_name}_var")
        scale = self.speed_scales[axis_name]
        low = int(scale.cget("from"))
        high = int(scale.cget("to"))
        try:
            value = int(float(raw))
        except ValueError:
            value = axis_var.get()
        axis_var.set(max(low, min(high, value)))
        self.axis_entry_vars[axis_name].set(str(axis_var.get()))

    def _on_mode_selected(self, _event: tk.Event | None = None):
        if self._mode_change_guard:
            return

        selected = self.mode_var.get()
        if selected.startswith("0:"):
            return

        confirmed = messagebox.askyesno(
            "确认切换模式",
            f"即将切换到底盘特殊模式：{selected}\n这可能接管当前运动控制。是否继续？",
        )
        if confirmed:
            return

        self._mode_change_guard = True
        try:
            self.mode_var.set("0: 正常控制")
            self.mode_cb.current(0)
        finally:
            self._mode_change_guard = False

    def _build_current_frame(self) -> MotionFrame:
        mode_str = self.mode_var.get()
        mode_val = int(mode_str.split(":")[0])
        return MotionFrame(
            mode=mode_val,
            vx=max(-32768, min(32767, self.vx_var.get())),
            vy=max(-32768, min(32767, self.vy_var.get())),
            vz=max(-32768, min(32767, self.vz_var.get())),
        )

    def _current_payload(self) -> tuple[int, int, int, int]:
        frame = self._build_current_frame()
        return (frame.mode, frame.vx, frame.vy, frame.vz)

    def _schedule_auto_send(self, reason: str = ""):
        if not self.auto_send_on_change_var.get():
            return
        if self.auto_send:
            return
        if not self.connected or self.ser is None or not self.ser.is_open:
            return

        payload = self._current_payload()
        if payload == self.last_sent_payload:
            return

        if self.auto_send_job is not None:
            try:
                self.root.after_cancel(self.auto_send_job)
            except tk.TclError:
                pass
            self.auto_send_job = None

        try:
            delay_ms = max(20, int(self.auto_send_delay_var.get()))
        except (ValueError, tk.TclError):
            delay_ms = 80
            self.auto_send_delay_var.set(delay_ms)

        self.auto_send_job = self.root.after(delay_ms, lambda r=reason: self._auto_send_if_needed(r))

    def _auto_send_if_needed(self, reason: str = ""):
        self.auto_send_job = None
        if self.auto_send:
            return
        if not self.auto_send_on_change_var.get():
            return
        if not self.connected or self.ser is None or not self.ser.is_open:
            return

        payload = self._current_payload()
        if payload == self.last_sent_payload:
            return

        self._send_once(auto_reason=reason or "auto")

    def _update_preview(self):
        self._set_frame_labels(self.byte_labels, self._build_current_frame().encode())

    def _set_frame_labels(self, labels: list[tk.Label], frame: bytes):
        highlight = {0: ACCENT2, 9: WARNING, 10: ACCENT2}
        speed_bytes = {3, 4, 5, 6, 7, 8}
        for index, label in enumerate(labels):
            value = frame[index] if index < len(frame) else 0
            if index in highlight:
                fg = highlight[index]
            elif index in speed_bytes:
                fg = ACCENT
            else:
                fg = TEXT_SEC
            label.config(text=f"{value:02X}", fg=fg)

    def _refresh_ports(self):
        current = self.port_var.get()
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if current in ports:
            self.port_var.set(current)
        elif ports:
            self.port_var.set(ports[0])
        else:
            self.port_var.set("")
        self.conn_btn.config(state="normal" if ports or self.connected else "disabled")
        self._log(f"已刷新串口列表: {', '.join(ports) if ports else '未发现设备'}", "inf")

    def _toggle_connection(self):
        if self.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("连接失败", "请先选择串口端口。")
            return

        try:
            baud = int(self.baud_var.get())
            timeout = max(0.01, float(self.timeout_var.get()))
        except ValueError:
            messagebox.showerror("参数错误", "波特率或超时时间格式不正确。")
            return

        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        except serial.SerialException as exc:
            messagebox.showerror("连接失败", str(exc))
            return

        self.connected = True
        self.rx_buffer.clear()
        self.last_sent_payload = None
        self.rx_count = 0
        self.rx_count_var.set("0")
        self.conn_btn.config(text="断开串口", style="Danger.TButton")
        self.rx_state_var.set("已连接，等待数据")
        self._refresh_runtime_indicators()
        self._log(f"已连接 {port}，波特率 {baud}，超时 {timeout:.2f}s", "inf")

        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()

    def _disconnect(self, reason: str | None = None):
        self.auto_send = False
        self.connected = False
        if self.auto_send_job is not None:
            try:
                self.root.after_cancel(self.auto_send_job)
            except tk.TclError:
                pass
            self.auto_send_job = None
        if self.ser is not None:
            try:
                if self.ser.is_open:
                    self.ser.close()
            except serial.SerialException:
                pass
            finally:
                self.ser = None

        self.auto_btn.config(text="连续发送", style="Secondary.TButton")
        self.conn_btn.config(text="连接串口", style="Primary.TButton")
        self.rx_state_var.set("串口已断开")
        self._refresh_runtime_indicators()

        log_msg = reason if reason else "串口已断开"
        self._log(log_msg, "warn" if reason else "inf")

        if reason and self.auto_reconnect_var.get():
            self.root.after(1500, self._auto_reconnect)

    def _auto_reconnect(self):
        if self.connected or self.ui_alive is False:
            return
        port = self.port_var.get().strip()
        if not port:
            return
        try:
            baud = int(self.baud_var.get())
            timeout = max(0.01, float(self.timeout_var.get()))
            self._log(f"尝试自动重连 {port} @ {baud}...", "inf")
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
            self.connected = True
            self.rx_buffer.clear()
            self.last_sent_payload = None
            self.rx_count = 0
            self.rx_count_var.set("0")
            self.conn_btn.config(text="断开串口", style="Danger.TButton")
            self.rx_state_var.set("重连成功，等待数据")
            self._refresh_runtime_indicators()
            self._log("自动重连成功", "inf")
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
        except serial.SerialException:
            self._log("自动重连失败，将在5秒后重试...", "warn")
            self.root.after(5000, self._auto_reconnect)

    def _read_loop(self):
        while self.connected and self.ser is not None:
            try:
                data = self.ser.read(128)
            except serial.SerialException as exc:
                self.root.after(0, lambda e=exc: self._disconnect(f"读取失败: {e}"))
                return

            if not data:
                continue

            self.rx_buffer.extend(data)
            while True:
                frame = self._extract_frame()
                if frame is None:
                    break
                self.root.after(0, lambda f=frame: self._handle_rx_frame(f))

    def _extract_frame(self) -> bytes | None:
        while self.rx_buffer and self.rx_buffer[0] not in (FRAME_HEAD, LOG_FRAME_HEAD):
            self.rx_buffer.pop(0)

        if not self.rx_buffer:
            return None

        if self.rx_buffer[0] == LOG_FRAME_HEAD:
            if len(self.rx_buffer) < LOG_FRAME_LEN:
                return None
            if self.rx_buffer[LOG_FRAME_LEN - 1] == LOG_FRAME_TAIL:
                frame = bytes(self.rx_buffer[:LOG_FRAME_LEN])
                del self.rx_buffer[:LOG_FRAME_LEN]
                return frame
            self.rx_buffer.pop(0)
            return None

        if len(self.rx_buffer) < TX_FRAME_LEN:
            return None

        if self.rx_buffer[10] == FRAME_TAIL:
            frame = bytes(self.rx_buffer[:TX_FRAME_LEN])
            del self.rx_buffer[:TX_FRAME_LEN]
            return frame

        if len(self.rx_buffer) >= RX_FRAME_LEN and self.rx_buffer[23] == FRAME_TAIL:
            frame = bytes(self.rx_buffer[:RX_FRAME_LEN])
            del self.rx_buffer[:RX_FRAME_LEN]
            return frame

        return None

    def _handle_rx_frame(self, frame: bytes):
        self.rx_count += 1
        self.rx_count_var.set(str(self.rx_count))
        self.last_rx_time = time.strftime("%H:%M:%S")
        self._set_frame_labels(self.last_rx_byte_labels, frame)

        if len(frame) == RX_FRAME_LEN:
            self._handle_status_frame(frame)
        elif len(frame) == LOG_FRAME_LEN:
            self._handle_log_frame(frame)
        elif len(frame) == TX_FRAME_LEN:
            self._handle_echo_frame(frame)
        else:
            self.rx_state_var.set(f"Unknown frame({len(frame)}B) @ {self.last_rx_time}")
            self._log(f"← RX: {frame_to_hex(frame)}  [Unknown frame]", "warn")

    def _handle_status_frame(self, frame: bytes):
        """解析24字节状态帧（机器人上行数据）"""
        try:
            status = StatusFrame.decode(frame)
        except Exception as e:
            self.rx_value_var.set(f"解析错误: {e}")
            self.rx_state_var.set(f"解析错误 @ {self.last_rx_time}")
            self._log(f"← RX: {frame_to_hex(frame)}  [解析错误: {e}]", "err")
            return

        battery_v = status.power_voltage / 1000.0
        vel_x_mps = scaled_to_unit(status.vel_x)
        vel_y_mps = scaled_to_unit(status.vel_y)
        vel_z_rads = scaled_to_unit(status.vel_z)
        self.rx_value_var.set(
            f"Vx={vel_x_mps:.3f} m/s  Vy={vel_y_mps:.3f} m/s  Vz={vel_z_rads:.3f} rad/s\n"
            f"电压={battery_v:.2f}V  停止={status.flag_stop}"
        )
        self.rx_state_var.set(
            f"BCC{'✓' if status.bcc_valid else '✗'}  {self.last_rx_time}"
        )

        self._update_dashboard(status)

        suffix = (
            f" 电池={battery_v:.2f}V"
            f" Vx={vel_x_mps:.3f}m/s Vy={vel_y_mps:.3f}m/s Vz={vel_z_rads:.3f}rad/s"
            f" Accel=({status.accel_x},{status.accel_y},{status.accel_z})"
            f" Gyro=({status.gyro_x},{status.gyro_y},{status.gyro_z})"
        )
        tag = "rx" if status.bcc_valid else "warn"
        self._log(f"← RX: {frame_to_hex(frame)}  [24B]{suffix}", tag)

        if self.log_writer:
            self.log_writer.writerow([
                time.time(), status.vel_x, status.vel_y, status.vel_z,
                status.accel_x, status.accel_y, status.accel_z,
                status.gyro_x, status.gyro_y, status.gyro_z,
                status.power_voltage, int(status.bcc_valid),
            ])

    def _handle_echo_frame(self, frame: bytes):
        """解析11字节回显帧。固件默认不保证回显，此处仅做兼容显示。"""
        valid = calc_bcc(frame[:9]) == frame[9]
        try:
            echo = MotionFrame(
                mode=frame[1],
                vx=struct.unpack(">h", frame[3:5])[0],
                vy=struct.unpack(">h", frame[5:7])[0],
                vz=struct.unpack(">h", frame[7:9])[0],
            )
            self.rx_value_var.set(
                f"回显: Vx={scaled_to_unit(echo.vx):.3f} m/s  "
                f"Vy={scaled_to_unit(echo.vy):.3f} m/s  "
                f"Vz={scaled_to_unit(echo.vz):.3f} rad/s"
            )
        except Exception:
            echo = None
            self.rx_value_var.set("回显解析失败")

        status = "BCC 正常" if valid else "BCC 错误"
        self.rx_state_var.set(f"回显 {status}  @ {self.last_rx_time}")

        tag = "rx" if valid else "warn"
        suffix = (
            f"  (Vx={scaled_to_unit(echo.vx):.3f}m/s"
            f" Vy={scaled_to_unit(echo.vy):.3f}m/s"
            f" Vz={scaled_to_unit(echo.vz):.3f}rad/s)"
            if echo else ""
        )
        self._log(f"← RX: {frame_to_hex(frame)}  [11B回显/兼容帧] {status}{suffix}", tag)

    def _handle_log_frame(self, frame: bytes):
        """Parse 19-byte device log frame."""
        try:
            log_frame = LogFrame.decode(frame)
        except Exception as e:
            self.rx_value_var.set(f"LOG parse error: {e}")
            self.rx_state_var.set(f"LOG frame parse error @ {self.last_rx_time}")
            self._log(f"← RX: {frame_to_hex(frame)}  [LOG parse error: {e}]", "err")
            return

        status = "BCC OK" if log_frame.bcc_valid else "BCC ERR"
        text = log_frame.text or "<empty>"
        self.rx_value_var.set(f"LOG[{log_frame.code:02X}] {text}")
        self.rx_state_var.set(f"LOG frame {status}  @ {self.last_rx_time}")

        tag = "err" if log_frame.bcc_valid else "warn"
        self._log(
            f"← RX: {frame_to_hex(frame)}  [LOG code=0x{log_frame.code:02X} text='{text}' {status}]",
            tag,
        )

    def _send_once(self, auto_reason: str | None = None):
        if not self.connected or self.ser is None or not self.ser.is_open:
            self._log("串口未连接，无法发送。", "err")
            return

        frame_obj = self._build_current_frame()
        frame = frame_obj.encode()
        try:
            self.ser.write(frame)
        except serial.SerialException as exc:
            self._disconnect(f"发送失败: {exc}")
            return

        self.last_sent_payload = (frame_obj.mode, frame_obj.vx, frame_obj.vy, frame_obj.vz)
        self.tx_count += 1
        self.tx_count_var.set(str(self.tx_count))
        send_ts = time.strftime("%H:%M:%S")
        self.last_send_var.set(send_ts)
        reason_text = f" [{auto_reason}]" if auto_reason else ""
        self._refresh_runtime_indicators()
        self._log(
            f"→ TX{reason_text}: {frame_to_hex(frame)}  "
            f"(Vx={scaled_to_unit(frame_obj.vx):.3f} m/s "
            f"Vy={scaled_to_unit(frame_obj.vy):.3f} m/s "
            f"Vz={scaled_to_unit(frame_obj.vz):.3f} rad/s)",
            "tx",
        )

    def _toggle_logging(self):
        if self.log_writer is not None:
            # 停止记录
            self.log_btn.config(text="开始记录", style="Secondary.TButton")
            if self.log_file:
                self.log_file.close()
            self.log_writer = None
            self.log_file = None
            self._refresh_runtime_indicators()
            self._log("数据记录已停止", "inf")
            return

        # 开始记录
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV 文件", "*.csv")],
            title="保存数据记录",
        )
        if not path:
            return

        try:
            self.log_file = open(path, "w", newline="")
            self.log_writer = csv.writer(self.log_file)
            self.log_writer.writerow([
                "timestamp", "vel_x", "vel_y", "vel_z",
                "accel_x", "accel_y", "accel_z",
                "gyro_x", "gyro_y", "gyro_z",
                "power_mv", "bcc_valid",
            ])
        except OSError as exc:
            messagebox.showerror("记录失败", str(exc))
            self.log_writer = None
            self.log_file = None
            return

        self.log_btn.config(text="停止记录", style="Danger.TButton")
        self._refresh_runtime_indicators()
        self._log(f"数据记录已开始: {os.path.basename(path)}", "inf")

    def _toggle_auto(self):
        if self.auto_send:
            self.auto_send = False
            self.auto_btn.config(text="连续发送", style="Secondary.TButton")
            self._refresh_runtime_indicators()
            self._log("连续发送已停止", "inf")
            return

        if not self.connected:
            self._log("请先连接串口，再启动连续发送。", "err")
            return

        try:
            interval = max(0.02, float(self.auto_interval_var.get()))
        except ValueError:
            self._log("发送周期格式不正确。", "err")
            return

        self.auto_send = True
        self.auto_btn.config(text="停止连发", style="Danger.TButton")
        self._refresh_runtime_indicators()
        self._log(f"连续发送已启动，周期 {interval:.2f}s", "inf")
        self.auto_thread = threading.Thread(target=self._auto_loop, daemon=True)
        self.auto_thread.start()

    def _auto_loop(self):
        while self.auto_send and self.connected:
            self.root.after(0, self._send_once)
            try:
                interval = max(0.02, float(self.auto_interval_var.get()))
            except (ValueError, tk.TclError):
                interval = 0.10
            time.sleep(interval)

    def _set_vel(self, vx_sign: int, vy_sign: int, vz_sign: int):
        step = self.step_var.get()
        if vx_sign:
            self.vx_var.set(vx_sign * step)
        if vy_sign:
            self.vy_var.set(vy_sign * step)
        if vz_sign:
            self.vz_var.set(vz_sign * 500)

    def _start_pointer_motion(self, key: str):
        self.active_pointer_keys.add(key)
        vx_sign, vy_sign, vz_sign = self.pointer_pad_map[key]
        self._set_vel(vx_sign, vy_sign, vz_sign)

    def _stop_pointer_motion(self, key: str):
        if key not in self.active_pointer_keys:
            return
        self.active_pointer_keys.discard(key)
        if not self.active_pointer_keys:
            self._zero_speeds()

    def _zero_speeds(self):
        self.vx_var.set(0)
        self.vy_var.set(0)
        self.vz_var.set(0)

    def _emergency_stop(self):
        self._zero_speeds()
        if self.connected:
            self._send_once()
        self._log("已执行急停", "warn")

    def _log(self, message: str, tag: str = "inf"):
        timestamp = time.strftime("%H:%M:%S")
        self.log_queue.put((f"[{timestamp}] {message}\n", tag))

    def _poll_log(self):
        if not self.ui_alive:
            return

        try:
            while True:
                text, tag = self.log_queue.get_nowait()
                self.log_text.config(state="normal")
                self.log_text.insert("end", text, tag)
                self.log_text.see("end")
                self.log_text.config(state="disabled")
        except queue.Empty:
            pass

        self.log_poll_job = self.root.after(100, self._poll_log)

    def _clear_log(self):
        self.log_text.config(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.config(state="disabled")

    def _on_close(self):
        self.ui_alive = False
        self.auto_send = False
        if self.layout_job is not None:
            try:
                self.root.after_cancel(self.layout_job)
            except tk.TclError:
                pass
            self.layout_job = None
        if self.log_poll_job is not None:
            try:
                self.root.after_cancel(self.log_poll_job)
            except tk.TclError:
                pass
            self.log_poll_job = None
        if self.connected:
            self._emergency_stop()
        self._disconnect()
        self.root.destroy()

    def _update_dashboard(self, status: StatusFrame):
        """更新IMU和电池仪表盘"""
        if hasattr(self, 'battery_var'):
            voltage = status.power_voltage / 1000.0
            self.battery_var.set(f"{voltage:.2f} V")
            try:
                cells = max(1, int(self.battery_cells_var.get().rstrip("S")))
            except ValueError:
                cells = 4
                self.battery_cells_var.set("4S")
            full_voltage = 4.2 * cells
            low_voltage = 3.0 * cells
            pct = max(0, min(100, (voltage - low_voltage) / max(0.1, full_voltage - low_voltage) * 100))
            self.battery_pct_var.set(f"{pct:.0f}%")
            battery_color = SUCCESS if pct >= 60 else WARNING if pct >= 30 else DANGER
            if "battery" in self.metric_value_labels:
                self.metric_value_labels["battery"].config(fg=battery_color)
            if hasattr(self, 'battery_bar'):
                self.battery_bar['value'] = pct

        if hasattr(self, 'imu_text'):
            self.imu_text.config(state="normal")
            self.imu_text.delete("1.0", "end")
            self.imu_text.insert("end", (
                f"Accel X: {status.accel_x:>6}\n"
                f"Accel Y: {status.accel_y:>6}\n"
                f"Accel Z: {status.accel_z:>6}\n"
                f"─────────────\n"
                f"Gyro  X: {status.gyro_x:>6}\n"
                f"Gyro  Y: {status.gyro_y:>6}\n"
                f"Gyro  Z: {status.gyro_z:>6}\n"
                f"─────────────\n"
                f"Vel X : {scaled_to_unit(status.vel_x):>6.3f} m/s\n"
                f"Vel Y : {scaled_to_unit(status.vel_y):>6.3f} m/s\n"
                f"Vel Z : {scaled_to_unit(status.vel_z):>6.3f} rad/s"
            ))
            self.imu_text.config(state="disabled")


def apply_dark_style():
    style = ttk.Style()
    style.theme_use("clam")
    style.configure(".", background=PANEL, foreground=TEXT_PRI)
    style.configure(
        "TButton",
        background=BTN_BG,
        foreground=TEXT_PRI,
        borderwidth=0,
        focusthickness=0,
        focuscolor=BTN_BG,
        padding=(10, 6),
    )
    style.map(
        "TButton",
        background=[("active", "#2b3138"), ("pressed", "#161b22")],
        foreground=[("disabled", TEXT_DIM)],
    )
    style.configure("Primary.TButton", background="#1f6feb", foreground="white")
    style.map(
        "Primary.TButton",
        background=[("active", "#388bfd"), ("pressed", "#1a56c4")],
    )
    style.configure("Secondary.TButton", background=BTN_BG, foreground=TEXT_PRI)
    style.map(
        "Secondary.TButton",
        background=[("active", "#30363d"), ("pressed", "#1c2128")],
    )
    style.configure("Danger.TButton", background="#7a2323", foreground="white")
    style.map(
        "Danger.TButton",
        background=[("active", "#b62324"), ("pressed", "#871e1f")],
    )
    style.configure(
        "Pad.TButton",
        background=BTN_BG,
        foreground=TEXT_PRI,
        padding=(8, 10),
        font=("Consolas", 14, "bold"),
    )
    style.map(
        "Pad.TButton",
        background=[("active", "#1f6feb"), ("pressed", "#194fb7")],
    )
    style.configure(
        "PadDanger.TButton",
        background="#6d1f1f",
        foreground="white",
        padding=(8, 10),
        font=("Consolas", 14, "bold"),
    )
    style.map(
        "PadDanger.TButton",
        background=[("active", DANGER), ("pressed", "#a41d1d")],
    )
    style.configure(
        "Ghost.TButton",
        background=PANEL,
        foreground=TEXT_SEC,
        padding=(8, 3),
    )
    style.map(
        "Ghost.TButton",
        background=[("active", "#1c2128"), ("pressed", "#11161d")],
        foreground=[("active", TEXT_PRI)],
    )
    style.configure(
        "Dark.TEntry",
        fieldbackground=BTN_BG,
        foreground=TEXT_PRI,
        insertcolor=TEXT_PRI,
        bordercolor=BORDER,
        lightcolor=BORDER,
        darkcolor=BORDER,
        padding=4,
    )
    style.configure(
        "TCombobox",
        fieldbackground=BTN_BG,
        background=BTN_BG,
        foreground=TEXT_PRI,
        selectbackground=BORDER,
        selectforeground=TEXT_PRI,
        bordercolor=BORDER,
        arrowcolor=TEXT_SEC,
    )
    style.map(
        "TCombobox",
        fieldbackground=[("readonly", BTN_BG)],
        foreground=[("readonly", TEXT_PRI)],
    )
    style.configure(
        "Dark.TCheckbutton",
        background=PANEL,
        foreground=TEXT_SEC,
        indicatorbackground=BTN_BG,
        indicatormargin=2,
    )
    style.map(
        "Dark.TCheckbutton",
        background=[("active", PANEL)],
        foreground=[("active", TEXT_PRI)],
        indicatorbackground=[("selected", ACCENT), ("!selected", BTN_BG)],
    )
    style.configure(
        "Battery.Horizontal.TProgressbar",
        troughcolor=BTN_BG,
        background=SUCCESS,
        bordercolor=BORDER,
        lightcolor=SUCCESS,
        darkcolor=SUCCESS,
    )
    style.configure("Vertical.TScrollbar", background=BTN_BG, troughcolor=BG, arrowcolor=TEXT_SEC)


if __name__ == "__main__":
    root = tk.Tk()
    apply_dark_style()
    app = WheeltecController(root)
    root.mainloop()
