#!/usr/bin/env python3
"""Unified operator GUI for feedback Leader Arm imitation-data collection.

Derived from the MIT-licensed ``fb_leaderarm`` GUI. See
``THIRD_PARTY_NOTICES.md``.
"""

from collections import deque, OrderedDict
from datetime import datetime
import json
import logging
from logging.handlers import RotatingFileHandler
import math
import os
from pathlib import Path
import sys
import threading
import time

import rclpy
from contact_observer_msgs.msg import ContactObservation, ObserverInput
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rcl_interfaces.msg import Log
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QDialogButtonBox, QGridLayout, QGroupBox,
    QHBoxLayout, QHeaderView, QLabel, QMainWindow, QMessageBox, QPushButton,
    QProgressBar, QSplitter, QTabWidget, QTableWidget, QTableWidgetItem,
    QTextEdit, QVBoxLayout, QWidget,
)


WINDOW_TITLE = "Feedback Leader Arm Teleoperation Data Collection"
MIB = 1024 * 1024


def _rss_bytes():
    try:
        page = os.sysconf("SC_PAGE_SIZE")
        return int(Path("/proc/self/statm").read_text().split()[1]) * int(page)
    except (OSError, ValueError, IndexError):
        return 0


def _fmt_bytes(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return "—"
    if value <= 0:
        return "—"
    return f"{value / MIB:.1f} MiB" if value < 1024 * MIB else f"{value / (1024*MIB):.2f} GiB"


def _quat_to_rpy_deg(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))
    sinp = max(-1.0, min(1.0, 2 * (w*y - z*x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


def _uint8_value(value):
    """Normalize ROS 2 uint8 values represented as int or one-byte bytes."""
    if isinstance(value, (bytes, bytearray)):
        if len(value) != 1:
            raise ValueError(f"expected one byte, got {len(value)}")
        return value[0]
    return int(value)


def _recorder_is_recording(status):
    """Fail safely when either the explicit flag or public state says recording."""
    if not isinstance(status, dict):
        return False
    return bool(status.get("recording")) or str(
        status.get("state", "")
    ).strip().upper() == "RECORDING"


class SignalPlot(QWidget):
    def __init__(self, title, unit, color):
        super().__init__()
        self.title, self.unit, self.color = title, unit, QColor(color)
        self.values = []
        self.setMinimumHeight(125)

    def set_values(self, values):
        self.values = list(values)
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#151a22"))
        painter.setPen(QColor("#dbe3ef"))
        latest = self.values[-1] if self.values else 0.0
        painter.drawText(8, 18, f"{self.title}   {latest:+.3f} {self.unit}")
        left, top, right, bottom = 8, 25, self.width()-8, self.height()-10
        painter.setPen(QColor("#343e4c"))
        painter.drawRect(left, top, max(1, right-left), max(1, bottom-top))
        if len(self.values) < 2:
            return
        lo, hi = min(self.values), max(self.values)
        margin = max(0.1, (hi-lo) * 0.1)
        lo, hi = lo-margin, hi+margin
        span = max(1e-9, hi-lo)
        painter.setPen(QPen(self.color, 1.4))
        last = None
        count = len(self.values)
        for index, value in enumerate(self.values):
            x = left + index * (right-left) / max(1, count-1)
            y = bottom - (value-lo) * (bottom-top) / span
            point = (int(x), int(y))
            if last is not None:
                painter.drawLine(last[0], last[1], point[0], point[1])
            last = point
        painter.setPen(QColor("#8996a8"))
        painter.drawText(left+3, top+13, f"{hi:+.2f}")
        painter.drawText(left+3, bottom-3, f"{lo:+.2f}")


class EventStore:
    def __init__(self, log_dir):
        self.events = deque(maxlen=1000)
        self.lock = threading.Lock()
        self.sequence = 0
        self.last_key_time = OrderedDict()
        Path(log_dir).mkdir(parents=True, exist_ok=True)
        self.logger = logging.getLogger(f"collection_events_{id(self)}")
        self.logger.setLevel(logging.INFO)
        self.logger.propagate = False
        handler = RotatingFileHandler(
            str(Path(log_dir) / "events.jsonl"), maxBytes=10*MIB,
            backupCount=4, encoding="utf-8")
        handler.setFormatter(logging.Formatter("%(message)s"))
        self.logger.addHandler(handler)

    def add(self, severity, source, state, episode, message, action="", dedup_sec=0.0):
        now = time.time()
        key = (severity, source, message)
        if not action:
            action = {
                "Camera": "USB 연결·전원·serial을 확인하세요. Recorder는 계속 재시도합니다.",
                "Recorder": "Data Health의 stale source와 episode pending 상태를 확인하세요.",
                "Writer": "저장 장치 쓰기 속도와 queue/RAM 사용량을 확인하세요.",
                "Teleop": "현재 단계의 readiness와 joint tolerance 차단 사유를 확인하세요.",
                "Observer": "모델·baseline·FREE 상태와 message freshness를 확인하세요.",
                "GUI": "ROS service/node가 online인지 확인한 뒤 다시 시도하세요.",
                "Launch": "해당 node의 rosout와 launch preflight 출력을 확인하세요.",
            }.get(source, "상세 로그와 source 상태를 확인하세요.")
        event = {
            "timestamp": datetime.fromtimestamp(now).astimezone().isoformat(timespec="milliseconds"),
            "severity": severity, "source": source, "teleop_state": state or "—",
            "episode": episode or "—", "message": str(message),
            "suggested_action": action,
        }
        with self.lock:
            if dedup_sec > 0 and now-self.last_key_time.get(key, 0.0) < dedup_sec:
                return None
            self.last_key_time[key] = now
            self.last_key_time.move_to_end(key)
            while len(self.last_key_time) > 1000:
                self.last_key_time.popitem(last=False)
            self.events.append(event)
            self.sequence += 1
            self.logger.info(json.dumps(event, ensure_ascii=False, separators=(",", ":")))
        return event

    def snapshot(self):
        with self.lock:
            return list(self.events), self.sequence

    def clear_view(self):
        with self.lock:
            self.events.clear()
            self.last_key_time.clear()
            self.sequence += 1


class CollectionRosNode(Node):
    def __init__(self, on_event):
        super().__init__("feedback_leaderarm_data_collection_gui")
        defaults = {
            "teleop_status_topic": "/leader_teleop_node/status",
            "recorder_diagnostics_topic": "/chem_acp_raw_data_collection/diagnostics",
            "display_ft_topic": "/aft_sensor2/wrench",
            "contact_observation_topic": "/contact_observer/right/observation",
            "observer_input_topic": "/bae_r/observer_input",
            "observer_diagnostics_topic": "/contact_observer_node/diagnostics",
            "command_pose_topic": "/right_dsr_controller/task_space_command",
            "collection_log_dir": "/tmp/feedback_leaderarm_collection_logs",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.on_event = on_event
        self.teleop = {}
        self.teleop_received = 0.0
        self.recorder = {}
        self.recorder_received = 0.0
        self.observer_diagnostics = {}
        self.observer_diagnostics_received = 0.0
        self.current_pose = None
        self.desired_pose = None
        self.command_pose = None
        self.contact = {"label": "WAITING", "received": 0.0}
        self.ft = deque(maxlen=6000)
        self.ft_lock = threading.Lock()
        self.pending_services = []
        sensor_qos = rclpy.qos.QoSProfile(depth=1)
        sensor_qos.reliability = rclpy.qos.ReliabilityPolicy.BEST_EFFORT
        # Isolate operator/health status from continuously-ready sensor data.
        self.status_callback_group = MutuallyExclusiveCallbackGroup()
        self.sensor_callback_group = MutuallyExclusiveCallbackGroup()
        p = lambda name: str(self.get_parameter(name).value)
        self.create_subscription(
            String, p("teleop_status_topic"), self._teleop_cb, 10,
            callback_group=self.status_callback_group)
        self.create_subscription(
            String, p("recorder_diagnostics_topic"), self._recorder_cb, 10,
            callback_group=self.status_callback_group)
        self.create_subscription(
            String, p("observer_diagnostics_topic"), self._observer_diag_cb, 10,
            callback_group=self.status_callback_group)
        self.create_subscription(
            WrenchStamped, p("display_ft_topic"), self._ft_cb, sensor_qos,
            callback_group=self.sensor_callback_group)
        self.create_subscription(
            ContactObservation, p("contact_observation_topic"), self._contact_cb,
            sensor_qos, callback_group=self.sensor_callback_group)
        self.create_subscription(
            ObserverInput, p("observer_input_topic"), self._input_cb, sensor_qos,
            callback_group=self.sensor_callback_group)
        self.create_subscription(
            PoseStamped, p("command_pose_topic"), self._command_cb, sensor_qos,
            callback_group=self.sensor_callback_group)
        self.create_subscription(Log, "/rosout", self._rosout_cb, 100)
        self.service_clients = {}
        teleop = "/leader_teleop_node/command/"
        recorder = "/chem_acp_raw_data_collection/"
        for key, service in {
            "c": teleop+"current", "t": teleop+"slow", "o": teleop+"fast",
            "z": teleop+"init_pose", "s": teleop+"pause", "r": teleop+"realign",
            "q": teleop+"shutdown", "1": recorder+"start_episode",
            "save": recorder+"stop_save", "discard": recorder+"stop_discard",
            "p": recorder+"recover", "0": recorder+"shutdown",
        }.items():
            self.service_clients[key] = self.create_client(Trigger, service)

    @staticmethod
    def _json(message):
        value = json.loads(message.data)
        if not isinstance(value, dict):
            raise ValueError("JSON root is not an object")
        return value

    def _teleop_cb(self, message):
        try:
            self.teleop = self._json(message)
            self.teleop_received = time.monotonic()
            if self.teleop.get("dxl_fault"):
                self.on_event(
                    "ERROR", "Teleop",
                    "Leader DXL fault: " + str(
                        self.teleop.get("dxl_fault_reason", "unknown")),
                    dedup_sec=3.0,
                )
        except Exception as exc:
            self.on_event("ERROR", "Teleop", f"teleop status JSON 오류: {exc}")

    def _recorder_cb(self, message):
        try:
            self.recorder = self._json(message)
            self.recorder_received = time.monotonic()
        except Exception as exc:
            self.on_event("ERROR", "Recorder", f"recorder diagnostics JSON 오류: {exc}")
            return
        error = str(self.recorder.get("last_error", ""))
        if error:
            self.on_event("ERROR", "Recorder", error, dedup_sec=3.0)
        for camera, error in self.recorder.get("camera_retry_errors", {}).items():
            if error:
                self.on_event("WARN", "Camera", f"Camera {camera}: {error}", dedup_sec=10.0)
        for row in self.recorder.get("modalities", []):
            if row.get("ok") is False:
                self.on_event(
                    "WARN", "Recorder",
                    f"{row.get('name', 'source')}: {row.get('reason', 'not ready')}",
                    dedup_sec=5.0)

    def _observer_diag_cb(self, message):
        try:
            self.observer_diagnostics = self._json(message)
            self.observer_diagnostics_received = time.monotonic()
        except Exception as exc:
            self.on_event("WARN", "Observer", f"observer diagnostics JSON 오류: {exc}", dedup_sec=5.0)

    def _ft_cb(self, msg):
        w = msg.wrench
        with self.ft_lock:
            self.ft.append((time.monotonic(), w.force.x, w.force.y, w.force.z,
                            w.torque.x, w.torque.y, w.torque.z))

    def _contact_cb(self, msg):
        if not msg.model_ready:
            label = "MODEL NOT READY"
        elif not msg.valid:
            label = "INVALID"
            self.on_event("WARN", "Observer", "ContactObservation invalid", dedup_sec=5.0)
        elif int(msg.contact_state) == int(ContactObservation.CONTACT):
            label = "CONTACT"
        else:
            label = "FREE"
        self.contact = {"label": label, "received": time.monotonic()}

    def _input_cb(self, msg):
        self.current_pose = list(msg.current_pose)
        self.desired_pose = list(msg.desired_pose)

    def _command_cb(self, msg):
        p = msg.pose.position
        # single_impedance_pose_publisher publishes this PoseStamped position
        # in millimetres, matching the ObserverInput pose convention and this
        # table's "mm / deg" label.
        self.command_pose = [p.x, p.y, p.z] + _quat_to_rpy_deg(msg.pose.orientation)

    def _rosout_cb(self, msg):
        level = _uint8_value(msg.level)
        if level < _uint8_value(Log.WARN):
            return
        severity = "FATAL" if level >= _uint8_value(Log.FATAL) else \
            "ERROR" if level >= _uint8_value(Log.ERROR) else "WARN"
        haystack = f"{msg.name} {msg.msg}".lower()
        source = "Camera" if "camera" in haystack or "realsense" in haystack else \
            "Recorder" if "record" in haystack or "writer" in haystack else \
            "Observer" if "observer" in haystack else \
            "Teleop" if "teleop" in haystack or "leader" in haystack else "Launch"
        self.on_event(severity, source, f"[{msg.name}] {msg.msg}", dedup_sec=1.0)

    def call(self, key, done):
        client = self.service_clients[key]
        if len(self.pending_services) >= 20:
            done(False, "pending ROS service 요청이 너무 많습니다.")
            return
        if not client.service_is_ready():
            done(False, f"ROS service offline: {client.srv_name}")
            return
        future = client.call_async(Trigger.Request())
        timeout_sec = 30.0 if key in ("r", "q", "z", "p") else 10.0
        self.pending_services.append(
            (future, done, client.srv_name, time.monotonic(), timeout_sec))

    def poll_futures(self):
        remaining = []
        for future, done, name, started, timeout_sec in self.pending_services:
            if not future.done():
                if time.monotonic() - started > timeout_sec:
                    future.cancel()
                    done(False, f"ROS service timeout: {name}")
                else:
                    remaining.append((future, done, name, started, timeout_sec))
                continue
            try:
                response = future.result()
                done(bool(response.success), str(response.message))
            except Exception as exc:
                done(False, f"{name}: {exc}")
        self.pending_services = remaining


class MainWindow(QMainWindow):
    def __init__(self, node, event_store):
        super().__init__()
        self.node, self.event_store = node, event_store
        self.setWindowTitle(WINDOW_TITLE)
        self.resize(1550, 950)
        self.visible_events = []
        self.last_event_sequence = -1
        self.started_monotonic = time.monotonic()
        self.status_labels = {}
        self._build_ui()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)

    def _badge(self, text="WAITING"):
        label = QLabel(text)
        label.setAlignment(Qt.AlignCenter)
        label.setMinimumWidth(125)
        label.setStyleSheet("padding:8px;border-radius:5px;background:#394250;color:white;font-weight:bold")
        return label

    def _build_ui(self):
        root = QWidget(); layout = QVBoxLayout(root)
        title = QLabel(WINDOW_TITLE); title.setStyleSheet("font-size:22px;font-weight:bold")
        layout.addWidget(title)
        status = QHBoxLayout()
        for key, name in [("contact","Contact"),("teleop","Teleop"),("recorder","Recorder"),
                          ("overall","System"),("episodes","Episodes")]:
            box = QVBoxLayout(); box.addWidget(QLabel(name)); self.status_labels[key] = self._badge(); box.addWidget(self.status_labels[key]); status.addLayout(box)
        status.addStretch(1); layout.addLayout(status)
        self.next_label = QLabel("다음 단계 조건을 기다리는 중입니다.")
        self.next_label.setStyleSheet("padding:8px;background:#222a35;color:#dbe3ef")
        layout.addWidget(self.next_label)
        controls = QHBoxLayout()
        for key, text in [("c","CURRENT (c)"),("t","SLOW (t)"),("o","FAST (o)"),
                          ("z","INIT POSE (z)"),("s","PAUSE (s)"),("r","REALIGN (r)"),("q","SHUTDOWN (q)"),
                          ("1","START EPISODE (1)"),("2","STOP (2)"),("p","RECOVER (p)"),("0","RECORDER EXIT (0)")]:
            button = QPushButton(text); button.clicked.connect(
                lambda _checked=False, k=key: self.command(k, "gui_button")); controls.addWidget(button)
        layout.addLayout(controls)
        tabs = QTabWidget(); tabs.addTab(self._dashboard_tab(), "Dashboard"); tabs.addTab(self._health_tab(), "Data Health"); tabs.addTab(self._logs_tab(), "Problem Logs")
        layout.addWidget(tabs, 1); self.setCentralWidget(root)

    def _dashboard_tab(self):
        widget = QWidget(); layout = QVBoxLayout(widget); split = QSplitter(Qt.Vertical)
        top = QWidget(); top_layout = QHBoxLayout(top)
        joint_box = QGroupBox("Leader / Follower joints (rad)"); jlay = QVBoxLayout(joint_box)
        self.joint_table = QTableWidget(6, 5); self.joint_table.setHorizontalHeaderLabels(["Joint","Leader","Mapped target","Follower","Error"])
        self.joint_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        for row in range(6): self.joint_table.setItem(row, 0, QTableWidgetItem(f"J{row+1}"))
        jlay.addWidget(self.joint_table); top_layout.addWidget(joint_box, 3)
        pose_box = QGroupBox("TCP poses (mm / deg)"); play = QVBoxLayout(pose_box)
        self.pose_table = QTableWidget(3, 7); self.pose_table.setHorizontalHeaderLabels(["Source","X","Y","Z","Rx","Ry","Rz"])
        self.pose_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        for row, source in enumerate(["Current TCP","Controller Desired","Teleop Command"]): self.pose_table.setItem(row,0,QTableWidgetItem(source))
        play.addWidget(self.pose_table); top_layout.addWidget(pose_box, 4)
        split.addWidget(top)
        plots = QWidget(); grid = QGridLayout(plots); self.plots = []
        colors = ["#4fc3f7","#66bb6a","#ef5350","#ab47bc","#ffa726","#ec407a"]
        for i, (name, unit) in enumerate(zip(["Fx","Fy","Fz","Mx","My","Mz"],["N"]*3+["Nm"]*3)):
            plot = SignalPlot(name, unit, colors[i]); self.plots.append(plot); grid.addWidget(plot, i//3, i%3)
        split.addWidget(plots); split.setSizes([310,420]); layout.addWidget(split)
        return widget

    def _health_tab(self):
        widget = QWidget(); layout = QVBoxLayout(widget)
        self.memory_label = QLabel(); self.memory_label.setStyleSheet("font-family:monospace;padding:8px")
        layout.addWidget(self.memory_label)
        self.queue_progress = QProgressBar(); layout.addWidget(self.queue_progress)
        self.modality_table = QTableWidget(0, 5); self.modality_table.setHorizontalHeaderLabels(["Modality","Target Hz","Actual Hz","Age","Status"])
        self.modality_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch); layout.addWidget(self.modality_table)
        self.camera_label = QLabel(); layout.addWidget(self.camera_label); return widget

    def _logs_tab(self):
        widget = QWidget(); layout = QVBoxLayout(widget); row = QHBoxLayout()
        self.log_filter = QComboBox(); self.log_filter.addItems(["All","Error","Warning","Camera","Recorder","Teleop","Observer"]); self.log_filter.currentTextChanged.connect(self._rebuild_logs)
        clear = QPushButton("Clear View"); clear.clicked.connect(self._clear_log_view); copy = QPushButton("Copy Details"); copy.clicked.connect(self._copy_log)
        row.addWidget(QLabel("Filter")); row.addWidget(self.log_filter); row.addStretch(1); row.addWidget(copy); row.addWidget(clear); layout.addLayout(row)
        self.log_table = QTableWidget(0,7); self.log_table.setHorizontalHeaderLabels(["Timestamp","Severity","Source","Teleop","Episode","Message","Suggested action"])
        self.log_table.horizontalHeader().setSectionResizeMode(5,QHeaderView.Stretch); self.log_table.itemSelectionChanged.connect(self._log_selection); layout.addWidget(self.log_table,3)
        self.log_details = QTextEdit(); self.log_details.setReadOnly(True); layout.addWidget(self.log_details,1); return widget

    def add_event(self, event):
        if event is not None: self._rebuild_logs()

    def command(self, key, input_source="gui"):
        if key in "ctozsrq":
            self._event(
                "INFO", "GUI",
                f"control command source={input_source} key={key} phase=request")
        if key == "z" and _recorder_is_recording(self.node.recorder):
            message = (
                "Recorder가 RECORDING 중입니다. 먼저 2로 episode를 저장한 뒤 "
                "INIT POSE로 이동하세요."
            )
            self._event("WARN", "GUI", message, dedup_sec=1.0)
            QMessageBox.warning(self, "INIT POSE 차단", message)
            return
        if key == "p":
            rec = self.node.recorder
            if _recorder_is_recording(rec):
                message = "Recorder가 RECORDING 중입니다. 먼저 2로 episode를 종료하세요."
                self._event("WARN", "GUI", message, dedup_sec=1.0)
                QMessageBox.warning(self, "복구 차단", message)
                return
            if bool(rec.get("draining")):
                message = "Recorder가 저장 queue를 비우는 중입니다. DRAINING이 끝난 뒤 p를 누르세요."
                self._event("WARN", "GUI", message, dedup_sec=1.0)
                QMessageBox.warning(self, "복구 대기", message)
                return
            if bool(rec.get("pending")):
                answer = QMessageBox.question(
                    self,
                    "Recorder 복구",
                    "문제가 발생한 pending episode를 폐기하고 현재 source를 다시 검사합니다. 계속할까요?",
                    QMessageBox.Yes | QMessageBox.No,
                    QMessageBox.No,
                )
                if answer != QMessageBox.Yes:
                    return
            self._call("p", input_source)
            return
        if key == "2":
            box = QMessageBox(self); box.setWindowTitle("Episode 종료"); box.setText("현재 episode를 어떻게 처리할까요?")
            save = box.addButton("중지 후 저장", QMessageBox.AcceptRole); discard = box.addButton("중지 후 폐기", QMessageBox.DestructiveRole); box.addButton("취소", QMessageBox.RejectRole); box.exec_()
            if box.clickedButton() is save: self._call("save", input_source)
            elif box.clickedButton() is discard: self._call("discard", input_source)
            return
        self._call(key, input_source)

    def _call(self, key, input_source="gui"):
        self.node.call(
            key,
            lambda ok, msg, k=key, source=input_source:
                self._service_done(k, ok, msg, source))

    def _service_done(self, key, ok, message, input_source="gui"):
        if key in "ctozsrq":
            result = "queued" if ok else "rejected"
            self._event(
                "INFO" if ok else "WARN", "GUI",
                f"control command source={input_source} key={key} "
                f"phase=service_result result={result}: {message}")
        if not ok:
            self._event("WARN", "GUI", f"명령 {key}: {message}")
            QMessageBox.warning(self, "명령 거부", message)

    def _event(self, severity, source, message, dedup_sec=0.0, severity_override=None):
        event = self.event_store.add(severity_override or severity, source,
            self.node.teleop.get("state"), self.node.recorder.get("episode"), message,
            dedup_sec=dedup_sec)
        self.add_event(event)

    def keyPressEvent(self, event):
        text = event.text().lower()
        if text in "ctozsrqp012": self.command(text, "gui_keyboard"); event.accept(); return
        super().keyPressEvent(event)

    def refresh(self):
        self.node.poll_futures()
        teleop, rec = self.node.teleop, self.node.recorder
        now = time.monotonic()
        contact = self.node.contact.get("label","WAITING")
        if now-self.node.contact.get("received",0.0) > 0.25 and contact not in ("WAITING","MODEL NOT READY"): contact = "STALE"
        state = teleop.get("state","WAITING")
        teleop_missing = (
            not self.node.teleop_received
            or now-self.node.teleop_received > 2.0
        )
        recorder_missing = (
            not self.node.recorder_received
            or now-self.node.recorder_received > 3.0
        )
        observer_missing = (
            not self.node.observer_diagnostics_received
            or now-self.node.observer_diagnostics_received > 2.5
        )
        observer_diag = self.node.observer_diagnostics
        observer_gate_ready = (
            not observer_missing
            and contact == "FREE"
            and bool(observer_diag.get("model_ready", False))
            and bool(observer_diag.get("baseline_ready", False))
            and (
                not bool(observer_diag.get(
                    "residual_bias_calibration_enabled", False))
                or bool(observer_diag.get("residual_bias_ready", False))
            )
        )
        recorder_offline = recorder_missing and (
            bool(self.node.recorder_received)
            or (
                observer_gate_ready
                and now-self.started_monotonic >= 5.0
            )
        )
        rstate = (
            "OFFLINE" if recorder_offline
            else rec.get("state", "WAITING OBSERVER")
        )
        memory = rec.get("memory",{}); modality_problem = any(row.get("ok") is False for row in rec.get("modalities", []))
        dxl_fault = bool(teleop.get("dxl_fault", False))
        core_status_missing = teleop_missing or observer_missing or recorder_offline
        if core_status_missing:
            overall = "WAITING" if now-self.started_monotonic < 5.0 else "ERROR"
        elif recorder_missing:
            overall = "WAITING"
        elif dxl_fault:
            overall = "ERROR"
        elif memory.get("level")=="CRITICAL" or "ERROR" in str(rstate):
            overall = "CRITICAL"
        elif memory.get("level")=="WARNING" or rec.get("last_error") or modality_problem:
            overall = "WARNING"
        else:
            overall = "OK"
        episodes = f"{rec.get('clean_saved_episodes',0)} clean / {rec.get('diagnostic_saved_episodes',0)} diag"
        for key,value in [("contact",contact),("teleop",state),("recorder",rstate),("overall",overall),("episodes",episodes)]: self._set_badge(key,value)
        blockers = teleop.get("blockers",[]); next_cmd = teleop.get("next_command","")
        self.next_label.setText((f"{next_cmd} 진입 가능 — 해당 버튼/키를 누르세요" if teleop.get("next_ready") else f"{next_cmd or 'NEXT'} BLOCKED — " + (", ".join(blockers) or "status 대기 중")))
        self.next_label.setStyleSheet("padding:8px;background:%s;color:white" % ("#16784a" if teleop.get("next_ready") else "#8a3c30"))
        self._refresh_joints(teleop); self._refresh_poses(); self._refresh_plots(); self._refresh_health(rec)
        self._check_timeouts_and_pressure(contact, rec)
        _events, sequence = self.event_store.snapshot()
        if sequence != self.last_event_sequence:
            self.last_event_sequence = sequence
            self._rebuild_logs()

    def _check_timeouts_and_pressure(self, contact, rec):
        now = time.monotonic()
        if now-self.started_monotonic < 5.0:
            return
        if not self.node.teleop_received or now-self.node.teleop_received > 2.0:
            self._event("ERROR", "Teleop", "Leader teleop status timeout/process offline", dedup_sec=10.0)
        if not self.node.observer_diagnostics_received or now-self.node.observer_diagnostics_received > 2.5:
            self._event("ERROR", "Observer", "Observer diagnostics timeout/process offline", dedup_sec=10.0)
        if contact in ("FREE", "CONTACT") and (
                not self.node.recorder_received or now-self.node.recorder_received > 3.0):
            self._event("ERROR", "Recorder", "Recorder diagnostics timeout/process offline", dedup_sec=10.0)
        if contact == "STALE":
            self._event("WARN", "Observer", "ContactObservation stale", dedup_sec=5.0)
        memory = rec.get("memory", {})
        if memory.get("level") in ("WARNING", "CRITICAL"):
            reasons = memory.get("critical_reasons") or memory.get("warning_reasons") or []
            self._event("ERROR" if memory.get("level")=="CRITICAL" else "WARN", "Recorder",
                        "; ".join(reasons) or f"Memory {memory.get('level')}", dedup_sec=5.0)
        queue = rec.get("queue", {})
        item_ratio = float(queue.get("items",0)) / max(1.0,float(queue.get("capacity",1)))
        byte_ratio = float(queue.get("bytes",0)) / max(1.0,float(queue.get("max_bytes",1)))
        warn_ratio = float(queue.get("warn_ratio", 0.75))
        if max(item_ratio, byte_ratio) >= warn_ratio:
            self._event("WARN", "Writer", f"Writer queue usage high: items={item_ratio:.1%}, bytes={byte_ratio:.1%}", dedup_sec=3.0)

    def _set_badge(self,key,value):
        good = str(value) in ("FREE","FAST","READY","RECORDING","OK")
        bad = (
            "ERROR" in str(value)
            or str(value) in (
                "CONTACT", "INVALID", "STALE", "MODEL NOT READY",
                "CRITICAL", "SHUTDOWN", "PENDING", "OFFLINE",
            )
        )
        color = "#178553" if good else "#a33b35" if bad else "#4b5668"
        self.status_labels[key].setText(str(value)); self.status_labels[key].setStyleSheet(f"padding:8px;border-radius:5px;background:{color};color:white;font-weight:bold")

    def _refresh_joints(self,status):
        vectors = [status.get("leader_rad"),status.get("mapped_target_rad"),status.get("follower_rad"),status.get("error_rad")]; over=status.get("over_tolerance",[False]*6)
        for row in range(6):
            for col,vec in enumerate(vectors,1):
                value = "—" if not isinstance(vec,list) or len(vec)<=row else f"{float(vec[row]):+.5f}"
                item = self.joint_table.item(row,col)
                if item is None:
                    item = QTableWidgetItem()
                    self.joint_table.setItem(row,col,item)
                item.setText(value)
                if col==4: item.setBackground(QColor("#a33b35" if row<len(over) and over[row] else "#176b48")); item.setForeground(QColor("white"))

    def _refresh_poses(self):
        for row,pose in enumerate([self.node.current_pose,self.node.desired_pose,self.node.command_pose]):
            for col in range(6):
                item = self.pose_table.item(row,col+1)
                if item is None:
                    item = QTableWidgetItem()
                    self.pose_table.setItem(row,col+1,item)
                item.setText("—" if pose is None else f"{float(pose[col]):+.3f}")

    def _refresh_plots(self):
        cutoff=time.monotonic()-20.0
        with self.node.ft_lock:
            while self.node.ft and self.node.ft[0][0]<cutoff: self.node.ft.popleft()
            rows=list(self.node.ft)
        for axis,plot in enumerate(self.plots,1): plot.set_values(row[axis] for row in rows)

    def _refresh_health(self,rec):
        mem=rec.get("memory",{}); queue=rec.get("queue",{}); observer_rss=self.node.observer_diagnostics.get("rss_bytes",0)
        self.memory_label.setText(
            f"Recorder RSS {_fmt_bytes(mem.get('rss_bytes'))} / {_fmt_bytes(mem.get('rss_hard_bytes'))}    "
            f"System MemAvailable {_fmt_bytes(mem.get('system_available_bytes'))}    GUI RSS {_fmt_bytes(_rss_bytes())}    "
            f"Observer RSS {_fmt_bytes(observer_rss)}    Memory {mem.get('level','—')}\n"
            f"Queue {queue.get('items',0)} / {queue.get('capacity',0)} items    "
            f"{_fmt_bytes(queue.get('bytes'))} / {_fmt_bytes(queue.get('max_bytes'))}    "
            f"peak {queue.get('peak_items',0)} items, {_fmt_bytes(queue.get('peak_bytes'))}")
        maximum=max(1,int(queue.get("capacity",1))); self.queue_progress.setMaximum(maximum); self.queue_progress.setValue(min(maximum,int(queue.get("items",0))))
        rows=rec.get("modalities",[]); self.modality_table.setRowCount(len(rows))
        for r,row in enumerate(rows):
            status = row.get("status", "OK" if row.get("ok") is True else row.get("reason", "—"))
            values=[row.get("name",row.get("modality","—")),row.get("target_hz","—"),row.get("actual_hz",row.get("hz","—")),row.get("age_s",row.get("age","—")),status]
            for c,value in enumerate(values): self.modality_table.setItem(r,c,QTableWidgetItem(str(value)))
        retries=rec.get("camera_retry_counts",{}); errors=rec.get("camera_retry_errors",{}); self.camera_label.setText(f"Camera retries: {retries}    active errors: {errors or 'none'}")

    def _matches(self,event):
        f=self.log_filter.currentText(); return f=="All" or (f=="Error" and event["severity"] in ("ERROR","FATAL")) or (f=="Warning" and event["severity"]=="WARN") or event["source"]==f

    def _rebuild_logs(self):
        events, sequence = self.event_store.snapshot(); self.last_event_sequence = sequence
        self.visible_events=[e for e in events if self._matches(e)]; self.log_table.setRowCount(len(self.visible_events))
        keys=["timestamp","severity","source","teleop_state","episode","message","suggested_action"]
        for row,event in enumerate(self.visible_events):
            for col,key in enumerate(keys): self.log_table.setItem(row,col,QTableWidgetItem(str(event[key])))
        if self.visible_events: self.log_table.scrollToBottom()

    def _clear_log_view(self): self.event_store.clear_view(); self.visible_events=[]; self.log_table.setRowCount(0); self.log_details.clear()
    def _log_selection(self):
        rows=self.log_table.selectionModel().selectedRows()
        if rows and rows[0].row()<len(self.visible_events): self.log_details.setPlainText(json.dumps(self.visible_events[rows[0].row()],ensure_ascii=False,indent=2))
    def _copy_log(self): QApplication.clipboard().setText(self.log_details.toPlainText())


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    app.setStyleSheet("QWidget{font-size:12px} QMainWindow{background:#eef1f5} QPushButton{padding:7px}")
    holder = {}
    def on_event(severity, source, message, dedup_sec=0.0):
        store=holder.get("store")
        if store is None: return
        store.add(severity,source,holder.get("node").teleop.get("state") if holder.get("node") else "—",holder.get("node").recorder.get("episode") if holder.get("node") else "—",message,dedup_sec=dedup_sec)
        # ROS callbacks are not allowed to update Qt widgets directly.  The GUI
        # timer observes EventStore.sequence and rebuilds the table safely.
    node=CollectionRosNode(on_event); holder["node"]=node
    store=EventStore(str(node.get_parameter("collection_log_dir").value)); holder["store"]=store
    window=MainWindow(node,store); holder["window"]=window; window.show()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    spin_stop = threading.Event()
    def spin_ros():
        while not spin_stop.is_set() and rclpy.ok():
            try:
                executor.spin_once(timeout_sec=0.1)
            except Exception as exc:
                if not rclpy.ok():
                    break
                on_event("ERROR", "GUI", f"ROS callback 오류: {exc}", dedup_sec=5.0)
    spin_thread = threading.Thread(target=spin_ros, name="collection_gui_ros", daemon=True)
    spin_thread.start()
    code=app.exec_()
    spin_stop.set()
    spin_thread.join(timeout=2.0)
    executor.shutdown(timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
