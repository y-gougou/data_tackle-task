#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
reorganize_data.py - 数据集整理工具

功能：
1. 将混杂的CSV和npy文件整理到规范目录结构
2. 按日期+故障类型分类
3. 分离raw和processed目录

使用方法：
  # 整理单个文件夹
  python reorganize_data.py --src "C:\数据集集合\0418"

  # 整理整个数据集集合
  python reorganize_data.py --src "C:\数据集集合"
"""

import os
import shutil
import argparse
from datetime import datetime


class DataReorganizer:
    """数据集整理器"""

    LABEL_MAP = {
        'normal': 0,
        '0': 0,
        'drive_fault': 1,
        'drive': 1,
        '1': 1,
        'wheel_slip': 2,
        'hub': 2,
        'loss': 2,
        '2': 2,
        'shaft_eccentric': 3,
        'shaft': 3,
        'eccentric': 3,
        '3': 3,
        'voltage_low': 4,
        'low_voltage': 4,
        'voltage': 4,
        '4': 4,
    }

    def __init__(self, src_dir):
        self.src_dir = src_dir

    def detect_label(self, filename):
        """从文件名检测故障标签"""
        filename_lower = filename.lower()

        for key, label in self.LABEL_MAP.items():
            if key in filename_lower:
                return label
        return 0  # 默认正常

    def detect_date(self, filename):
        """从文件名检测日期"""
        # 常见格式: 20260419, 2026-04-19, 2026_04_19
        import re
        patterns = [
            r'(\d{4}-\d{2}-\d{2})',  # 2026-04-19
            r'(\d{4}_\d{2}_\d{2})',   # 2026_04_19
            r'(\d{4})(\d{2})(\d{2})',  # 20260419
        ]

        for pattern in patterns:
            match = re.search(pattern, filename)
            if match:
                parts = match.groups()
                if len(parts) == 3:
                    if '-' in pattern or '_' in pattern:
                        return f"{parts[0]}-{parts[1]}-{parts[2]}"
                    else:
                        return f"{parts[0]}-{parts[1]}-{parts[2]}"

        return datetime.now().strftime("%Y-%m-%d")

    def reorganize(self):
        """执行整理"""
        print(f"整理目录: {self.src_dir}")

        # 遍历所有文件
        csv_files = []
        npy_files = []
        other_files = []

        for root, dirs, files in os.walk(self.src_dir):
            for f in files:
                filepath = os.path.join(root, f)
                ext = os.path.splitext(f)[1].lower()

                if ext == '.csv':
                    csv_files.append(filepath)
                elif ext == '.npy':
                    npy_files.append(filepath)
                else:
                    other_files.append((filepath, f))

        print(f"\n找到文件:")
        print(f"  CSV: {len(csv_files)}")
        print(f"  NPY: {len(npy_files)}")
        print(f"  其他: {len(other_files)}")

        # 按日期+标签分组
        groups = {}
        for csv_path in csv_files:
            filename = os.path.basename(csv_path)
            date = self.detect_date(filename)
            label = self.detect_label(filename)
            label_name = ['normal', 'drive_fault', 'wheel_slip', 'shaft_eccentric', 'voltage_low'][label]

            key = f"{date}_{label_name}"
            if key not in groups:
                groups[key] = {'csv': [], 'npy': [], 'others': []}

            groups[key]['csv'].append(csv_path)

        # 分配npy文件
        for npy_path in npy_files:
            filename = os.path.basename(npy_path)
            # 尝试从文件名推断属于哪个组
            found = False
            for key in groups:
                date_str = key.split('_')[0]
                if date_str in filename:
                    groups[key]['npy'].append(npy_path)
                    found = True
                    break
            if not found:
                # 放到第一个组
                if groups:
                    first_key = list(groups.keys())[0]
                    groups[first_key]['npy'].append(npy_path)

        # 分配其他文件
        for filepath, fname in other_files:
            if '_norm_params' in fname or '_stats' in fname:
                for key in groups:
                    if any(c in os.path.basename(groups[key]['csv'][0]) for c in fname.split('_')[:2]):
                        groups[key]['others'].append(filepath)
                        break
            elif '_info' in fname:
                for key in groups:
                    if 'w100' in fname and key.split('_')[0] in fname:
                        groups[key]['others'].append(filepath)
                        break
            else:
                for key in groups:
                    if key.split('_')[0] in filepath:
                        groups[key]['others'].append(filepath)
                        break

        # 创建目录并移动文件
        print(f"\n整理计划:")
        for key, files in groups.items():
            print(f"\n{key}/")
            if files['csv']:
                print(f"  CSV: {len(files['csv'])}")
            if files['npy']:
                print(f"  NPY: {len(files['npy'])}")
            if files['others']:
                print(f"  其他: {len(files['others'])}")

        # 执行移动
        print("\n" + "=" * 50)
        confirm = input("确认执行整理? (y/n): ")
        if confirm.lower() != 'y':
            print("取消整理")
            return

        for key, files in groups.items():
            # 创建目录
            raw_dir = os.path.join(self.src_dir, key, 'raw')
            proc_dir = os.path.join(self.src_dir, key, 'processed')
            os.makedirs(raw_dir, exist_ok=True)
            os.makedirs(proc_dir, exist_ok=True)

            # 移动CSV到raw
            for csv_path in files['csv']:
                dest = os.path.join(raw_dir, os.path.basename(csv_path))
                if not os.path.exists(dest):
                    shutil.move(csv_path, dest)
                    print(f"移动: {os.path.basename(csv_path)} → {key}/raw/")

            # 移动NPY到processed
            for npy_path in files['npy']:
                dest = os.path.join(proc_dir, os.path.basename(npy_path))
                if not os.path.exists(dest):
                    shutil.move(npy_path, dest)
                    print(f"移动: {os.path.basename(npy_path)} → {key}/processed/")

            # 移动其他文件到processed
            for other_path in files['others']:
                dest = os.path.join(proc_dir, os.path.basename(other_path))
                if not os.path.exists(dest):
                    shutil.move(other_path, dest)
                    print(f"移动: {os.path.basename(other_path)} → {key}/processed/")

        print("\n整理完成!")


def main():
    parser = argparse.ArgumentParser(description='数据集整理工具')
    parser.add_argument('--src', type=str, required=True,
                       help='源目录')
    args = parser.parse_args()

    reorganizer = DataReorganizer(args.src)
    reorganizer.reorganize()


if __name__ == '__main__':
    main()
