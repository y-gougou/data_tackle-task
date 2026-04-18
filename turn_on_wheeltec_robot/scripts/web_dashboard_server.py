#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Static file server for the web teleop dashboard, plus a lightweight
/api/data/ endpoint for listing and downloading recorded CSV files.
"""

import json
import os
import threading

import rospy

try:
    from http.server import SimpleHTTPRequestHandler
except ImportError:
    from SimpleHTTPServer import SimpleHTTPRequestHandler

try:
    from socketserver import TCPServer
except ImportError:
    from SocketServer import TCPServer

try:
    from urllib.parse import unquote
except ImportError:
    from urllib import unquote


class ReusableTCPServer(TCPServer):
    allow_reuse_address = True


class DashboardHandler(SimpleHTTPRequestHandler):

    def log_message(self, _format, *_args):
        return

    def do_GET(self):
        if self.path == "/api/data/list":
            self._handle_list()
        elif self.path.startswith("/api/data/download/"):
            self._handle_download()
        else:
            SimpleHTTPRequestHandler.do_GET(self)

    def _handle_list(self):
        d = self.server.data_dir
        if not d or not os.path.isdir(d):
            self._json_response({"files": [], "error": "data dir not found"})
            return
        entries = []
        for name in sorted(os.listdir(d), reverse=True):
            fp = os.path.join(d, name)
            if os.path.isfile(fp) and name.endswith(".csv"):
                entries.append({
                    "name": name,
                    "size": os.path.getsize(fp),
                    "mtime": os.path.getmtime(fp),
                })
        self._json_response({"files": entries, "dir": d})

    def _handle_download(self):
        name = unquote(self.path[len("/api/data/download/"):])
        if not name or ".." in name or "/" in name or "\\" in name:
            self._error_response(400, "invalid filename")
            return
        d = self.server.data_dir
        if not d:
            self._error_response(404, "data dir not configured")
            return
        fp = os.path.join(d, name)
        if not os.path.isfile(fp):
            self._error_response(404, "file not found")
            return
        size = os.path.getsize(fp)
        self.send_response(200)
        self.send_header("Content-Type", "text/csv")
        self.send_header("Content-Disposition", "attachment; filename=\"%s\"" % name)
        self.send_header("Content-Length", str(size))
        self.end_headers()
        with open(fp, "rb") as f:
            while True:
                chunk = f.read(65536)
                if not chunk:
                    break
                self.wfile.write(chunk)

    def _json_response(self, obj, code=200):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _error_response(self, code, msg):
        self._json_response({"error": msg}, code)


class StaticDashboardServer(object):
    def __init__(self):
        self.host = rospy.get_param("~host", "0.0.0.0")
        self.port = int(rospy.get_param("~port", 8000))
        self.data_dir = rospy.get_param("~data_dir", "/home/wheeltec/R550PLUS_data_collect/log")
        self.package_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.web_root = os.path.join(self.package_root, "web")

        if not os.path.isdir(self.web_root):
            raise RuntimeError("Web root does not exist: %s" % self.web_root)

        self.server = None
        self.server_thread = None

    def start(self):
        os.chdir(self.web_root)
        self.server = ReusableTCPServer((self.host, self.port), DashboardHandler)
        self.server.data_dir = self.data_dir
        self.server_thread = threading.Thread(target=self.server.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(
            "web dashboard started http://%s:%d (web=%s, data=%s)",
            self.host, self.port, self.web_root, self.data_dir,
        )

    def shutdown(self):
        if self.server is not None:
            try:
                self.server.shutdown()
                self.server.server_close()
            except Exception:
                pass
            self.server = None


if __name__ == "__main__":
    rospy.init_node("web_dashboard_server", anonymous=False)
    server = StaticDashboardServer()
    server.start()
    rospy.spin()
