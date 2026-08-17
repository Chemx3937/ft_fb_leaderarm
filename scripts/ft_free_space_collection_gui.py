#!/usr/bin/env python3
"""FT free-space wrench collection GUI derived from fb_leaderarm's GUI."""

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
from contact_observer_msgs.msg import ObserverInput
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rcl_interfaces.msg import Log
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from PyQt5.QtCore import QProcess, Qt, QTimer
from PyQt5.QtGui import QColor, QPainter, QPen, QTextCursor
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QGridLayout, QGroupBox,
    QHBoxLayout, QHeaderView, QLabel, QMainWindow, QMessageBox, QPushButton,
    QSplitter, QTabWidget, QTableWidget, QTableWidgetItem, QTextEdit,
    QVBoxLayout, QWidget,
)


WINDOW_TITLE = "FT Free-space Wrench Data Collection"
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


def _collector_is_collecting(status):
    """Return the collector's explicit episode state."""
    if not isinstance(status, dict):
        return False
    return bool(status.get("collecting"))


def _dataset_fingerprint(data_dir):
    return tuple(
        (path.name, path.stat().st_size, path.stat().st_mtime_ns)
        for path in sorted(Path(data_dir).expanduser().resolve().glob("*.npz"))
    )


def _pipeline_arguments(mode, data_dir, output_path):
    if mode == "validate":
        return "ft_free_space_validate", [
            "--data-dir", str(data_dir), "--output", str(output_path)]
    if mode == "train":
        return "ft_free_space_train", [
            "--data-dir", str(data_dir), "--output-dir", str(output_path)]
    raise ValueError(f"unknown pipeline mode: {mode}")


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
                "Collector": "Zero Gate와 collector service 상태를 확인하세요.",
                "Teleop": "현재 단계의 readiness와 joint tolerance 차단 사유를 확인하세요.",
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
        super().__init__("ft_free_space_collection_gui")
        defaults = {
            "teleop_status_topic": "/leader_teleop_node/status",
            "collector_diagnostics_topic": "/ft_free_space_collector/diagnostics",
            "display_ft_topic": "/aft_sensor2/wrench",
            "observer_input_topic": "/contact_state/observer_input",
            "command_pose_topic": "/right_dsr_controller/task_space_command",
            "collection_log_dir": "/tmp/ft_free_space_collection_gui_logs",
            "data_dir": str(Path.home() / ".ros/ft_fb_leaderarm/data"),
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.on_event = on_event
        self.teleop = {}
        self.teleop_received = 0.0
        self.collector = {}
        self.collector_received = 0.0
        self.current_pose = None
        self.desired_pose = None
        self.command_pose = None
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
            String, p("collector_diagnostics_topic"), self._collector_cb, 10,
            callback_group=self.status_callback_group)
        self.create_subscription(
            WrenchStamped, p("display_ft_topic"), self._ft_cb, sensor_qos,
            callback_group=self.sensor_callback_group)
        self.create_subscription(
            ObserverInput, p("observer_input_topic"), self._input_cb, sensor_qos,
            callback_group=self.sensor_callback_group)
        self.create_subscription(
            PoseStamped, p("command_pose_topic"), self._command_cb, sensor_qos,
            callback_group=self.sensor_callback_group)
        self.create_subscription(Log, "/rosout", self._rosout_cb, 100)
        self.service_clients = {}
        teleop = "/leader_teleop_node/command/"
        collector = "/ft_free_space_collector/"
        for key, service in {
            "c": teleop+"current", "t": teleop+"slow", "o": teleop+"fast",
            "z": teleop+"init_pose", "s": teleop+"pause", "r": teleop+"realign",
            "q": teleop+"shutdown", "1": collector+"start_episode",
            "2": collector+"stop_episode",
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

    def _collector_cb(self, message):
        try:
            self.collector = self._json(message)
            self.collector_received = time.monotonic()
        except Exception as exc:
            self.on_event(
                "ERROR", "Collector",
                f"collector diagnostics JSON 오류: {exc}", dedup_sec=5.0)

    def _ft_cb(self, msg):
        w = msg.wrench
        with self.ft_lock:
            self.ft.append((time.monotonic(), w.force.x, w.force.y, w.force.z,
                            w.torque.x, w.torque.y, w.torque.z))

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
        source = "Collector" if "collector" in haystack or "episode" in haystack else \
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
        timeout_sec = 30.0 if key in ("r", "q", "z") else 10.0
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
        self.pipeline_process = None
        self.validated_dataset = None
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
        for key, name in [("zero","Zero Gate"),("teleop","Teleop"),
                          ("collector","FT Collector"),("overall","System"),
                          ("samples","Samples")]:
            box = QVBoxLayout(); box.addWidget(QLabel(name)); self.status_labels[key] = self._badge(); box.addWidget(self.status_labels[key]); status.addLayout(box)
        status.addStretch(1); layout.addLayout(status)
        self.next_label = QLabel("다음 단계 조건을 기다리는 중입니다.")
        self.next_label.setStyleSheet("padding:8px;background:#222a35;color:#dbe3ef")
        layout.addWidget(self.next_label)
        guide = QLabel(
            "순서: POSITION 정렬 → AFT zero-set·Zero Gate VERIFIED → "
            "START FT EPISODE 성공 → CURRENT → SLOW 무접촉 → 접촉 전 STOP FT EPISODE\n"
            "CURRENT/SLOW/FAST는 FT Collector가 RECORDING일 때만 허용됩니다.")
        guide.setWordWrap(True)
        guide.setStyleSheet(
            "padding:8px;background:#594a1d;color:#fff3bf;font-weight:bold")
        layout.addWidget(guide)
        controls = QHBoxLayout()
        for key, text in [
                ("z","INIT POSE (z)"),("r","REALIGN (r)"),
                ("1","START FT EPISODE (1)"),("c","CURRENT (c)"),
                ("t","SLOW (t)"),("o","FAST (o)"),
                ("2","STOP FT EPISODE (2)"),("s","PAUSE (s)"),
                ("q","SHUTDOWN (q)")]:
            button = QPushButton(text); button.clicked.connect(
                lambda _checked=False, k=key: self.command(k, "gui_button")); controls.addWidget(button)
        layout.addLayout(controls)
        tabs = QTabWidget(); tabs.addTab(self._dashboard_tab(), "Dashboard"); tabs.addTab(self._health_tab(), "Data Health"); tabs.addTab(self._pipeline_tab(), "Dataset / Training"); tabs.addTab(self._logs_tab(), "Problem Logs")
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
        self.collector_details = QTextEdit()
        self.collector_details.setReadOnly(True)
        self.collector_details.setStyleSheet("font-family:monospace")
        layout.addWidget(self.collector_details)
        return widget

    def _pipeline_tab(self):
        widget = QWidget(); layout = QVBoxLayout(widget)
        self.data_dir = Path(
            str(self.node.get_parameter("data_dir").value)
        ).expanduser().resolve()
        info = QLabel(
            f"Dataset: {self.data_dir}\n"
            "수집 중이 아니고 leader가 정지 상태일 때만 실행됩니다. "
            "학습은 현재 dataset 검증을 통과한 뒤에만 허용됩니다.")
        info.setWordWrap(True); layout.addWidget(info)
        row = QHBoxLayout()
        self.validate_button = QPushButton("VALIDATE DATASET")
        self.train_button = QPushButton("TRAIN 5 ABLATIONS")
        self.validate_button.clicked.connect(
            lambda: self._start_pipeline("validate"))
        self.train_button.clicked.connect(lambda: self._start_pipeline("train"))
        self.train_button.setEnabled(False)
        self.pipeline_status = self._badge("IDLE")
        row.addWidget(self.validate_button); row.addWidget(self.train_button)
        row.addWidget(self.pipeline_status); row.addStretch(1); layout.addLayout(row)
        self.pipeline_output = QTextEdit(); self.pipeline_output.setReadOnly(True)
        self.pipeline_output.setStyleSheet("font-family:monospace")
        layout.addWidget(self.pipeline_output, 1)
        return widget

    def _logs_tab(self):
        widget = QWidget(); layout = QVBoxLayout(widget); row = QHBoxLayout()
        self.log_filter = QComboBox(); self.log_filter.addItems(["All","Error","Warning","Collector","Teleop","GUI","Pipeline","Launch"]); self.log_filter.currentTextChanged.connect(self._rebuild_logs)
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
        collector_fresh = (
            self.node.collector_received
            and time.monotonic() - self.node.collector_received <= 2.5)
        teleop_fresh = (
            self.node.teleop_received
            and time.monotonic() - self.node.teleop_received <= 2.0)
        collecting = _collector_is_collecting(self.node.collector)
        teleop_state = str(self.node.teleop.get("state", "WAITING"))
        if self.pipeline_process is not None and key in "1ctozr":
            message = (
                "Dataset 검증/학습이 실행 중입니다. 완료 후 수집 또는 이동을 "
                "시작하세요. STOP, PAUSE, SHUTDOWN은 계속 사용할 수 있습니다.")
            self._event("WARN", "GUI", message, dedup_sec=1.0)
            QMessageBox.warning(self, "Offline job 실행 중", message)
            return
        if key == "1" and (not teleop_fresh or teleop_state != "IDLE"):
            message = (
                "START FT EPISODE 차단: leader를 POSITION 정렬 완료 상태(IDLE)에 "
                f"두세요. 현재 Teleop 상태는 {teleop_state}입니다.")
            self._event("WARN", "GUI", message, dedup_sec=1.0)
            QMessageBox.warning(self, "FT 수집 시작 차단", message)
            return
        if key in ("c", "t", "o") and (
                not collector_fresh or not collecting):
            message = (
                "Teleoperation 차단: 먼저 Zero Gate VERIFIED 상태에서 START FT "
                "EPISODE가 성공하고 FT Collector 배지가 RECORDING인지 확인하세요."
            )
            self._event("WARN", "GUI", message, dedup_sec=1.0)
            QMessageBox.warning(self, "Teleoperation 차단", message)
            return
        if key in ("z", "r") and collecting:
            message = (
                "FT Collector가 RECORDING 중입니다. 먼저 2로 episode를 저장한 뒤 "
                "INIT POSE/REALIGN을 실행하세요.")
            self._event("WARN", "GUI", message, dedup_sec=1.0)
            QMessageBox.warning(self, "이동 명령 차단", message)
            return
        self._call(key, input_source)

    def _start_pipeline(self, mode):
        state = str(self.node.teleop.get("state", "WAITING"))
        if self.pipeline_process is not None:
            return
        if self.node.pending_services:
            message = "진행 중인 ROS 명령의 완료를 확인한 뒤 다시 실행하세요."
            self._event("WARN", "Pipeline", message)
            QMessageBox.warning(self, "Dataset 작업 차단", message)
            return
        if _collector_is_collecting(self.node.collector) or state not in (
                "IDLE", "PAUSED", "SHUTDOWN", "WAITING"):
            message = (
                "먼저 FT episode를 중지하고 leader를 IDLE/PAUSED/SHUTDOWN "
                f"상태로 두세요. 현재 Teleop 상태는 {state}입니다.")
            self._event("WARN", "Pipeline", message)
            QMessageBox.warning(self, "Dataset 작업 차단", message)
            return
        fingerprint = _dataset_fingerprint(self.data_dir)
        if mode == "train" and self.validated_dataset != (
                self.data_dir, fingerprint):
            message = "현재 dataset을 VALIDATE DATASET으로 먼저 검증하세요."
            self._event("WARN", "Pipeline", message)
            QMessageBox.warning(self, "학습 차단", message)
            return
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        output = (
            self.data_dir / f"dataset_validation_{stamp}.json"
            if mode == "validate" else
            self.data_dir / "models" / f"free_space_{stamp}")
        executable, arguments = _pipeline_arguments(mode, self.data_dir, output)
        script = Path(__file__).with_name(executable)
        if not script.is_file():
            message = f"실행 파일을 찾을 수 없습니다: {script}"
            self._event("ERROR", "Pipeline", message)
            QMessageBox.critical(self, "Dataset 작업 실패", message)
            return
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(
            lambda p=process: self._pipeline_output_ready(p))
        process.finished.connect(
            lambda code, _status, p=process, m=mode, d=self.data_dir,
                   o=output, f=fingerprint:
            self._pipeline_finished(p, m, d, o, f, code))
        self.pipeline_process = process
        self.pipeline_output.setPlainText(
            " ".join([sys.executable, str(script), *arguments]) + "\n")
        self.pipeline_status.setText(
            "VALIDATING" if mode == "validate" else "TRAINING")
        self._event("INFO", "Pipeline", f"{mode} started: {output}")
        process.start(sys.executable, [str(script), *arguments])
        if not process.waitForStarted(1000):
            message = process.errorString()
            self.pipeline_process = None
            process.deleteLater()
            self.pipeline_status.setText("FAILED")
            self._event("ERROR", "Pipeline", f"{mode} start failed: {message}")
            QMessageBox.critical(self, "Dataset 작업 실패", message)

    def _pipeline_output_ready(self, process):
        chunk = bytes(process.readAllStandardOutput()).decode(
            "utf-8", errors="replace")
        self.pipeline_output.moveCursor(QTextCursor.End)
        self.pipeline_output.insertPlainText(chunk)
        self.pipeline_output.ensureCursorVisible()

    def _pipeline_finished(
            self, process, mode, data_dir, output, fingerprint, exit_code):
        self._pipeline_output_ready(process)
        if self.pipeline_process is process:
            self.pipeline_process = None
        current = _dataset_fingerprint(data_dir)
        passed = exit_code == 0 and current == fingerprint
        if mode == "validate" and passed:
            self.validated_dataset = (data_dir, fingerprint)
            result = "VALIDATED"
        elif mode == "train" and exit_code == 0:
            result = "APPROVED"
        elif mode == "train" and exit_code == 2:
            result = "REJECTED"
        else:
            result = "FAILED"
        if current != fingerprint:
            result = "DATASET CHANGED"
            self.validated_dataset = None
        self.pipeline_status.setText(result)
        severity = "INFO" if passed else "WARN"
        self._event(
            severity, "Pipeline",
            f"{mode} finished result={result} exit={exit_code}: {output}")
        process.deleteLater()

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
        elif key == "1":
            self.validated_dataset = None
            QMessageBox.information(
                self, "FT 수집 시작 성공",
                "FT episode가 시작되었습니다. FT Collector 배지가 RECORDING으로 "
                "바뀐 뒤 CURRENT를 누르세요. 접촉 전 반드시 2로 중지하세요.")
        elif key == "2":
            QMessageBox.information(self, "FT 수집 종료", message)

    def _event(self, severity, source, message, dedup_sec=0.0, severity_override=None):
        event = self.event_store.add(severity_override or severity, source,
            self.node.teleop.get("state"),
            "ACTIVE" if _collector_is_collecting(self.node.collector) else "—",
            message,
            dedup_sec=dedup_sec)
        self.add_event(event)

    def keyPressEvent(self, event):
        text = event.text().lower()
        if text in "ctozsrq12": self.command(text, "gui_keyboard"); event.accept(); return
        super().keyPressEvent(event)

    def refresh(self):
        self.node.poll_futures()
        teleop, collector = self.node.teleop, self.node.collector
        now = time.monotonic()
        state = teleop.get("state","WAITING")
        teleop_missing = (
            not self.node.teleop_received
            or now-self.node.teleop_received > 2.0
        )
        collector_missing = (
            not self.node.collector_received
            or now-self.node.collector_received > 2.5)
        collecting = _collector_is_collecting(collector)
        zero = collector.get("zero", {})
        if not isinstance(zero, dict):
            zero = {}
        zero_ready = bool(zero.get("ready", False))
        zero_reason = str(zero.get("reason", "waiting_for_diagnostics"))
        zero_state = (
            "OFFLINE" if collector_missing else
            "LATCHED" if collecting else
            "VERIFIED" if zero_ready else zero_reason)
        collector_state = (
            "OFFLINE" if collector_missing else
            "RECORDING" if collecting else "IDLE")
        dxl_fault = bool(teleop.get("dxl_fault", False))
        if teleop_missing or collector_missing:
            overall = "WAITING" if now-self.started_monotonic < 5.0 else "ERROR"
        elif dxl_fault:
            overall = "ERROR"
        elif not collecting and not zero_ready:
            overall = "WAITING ZERO"
        else:
            overall = "OK"
        samples = str(collector.get("samples", 0))
        for key,value in [("zero",zero_state),("teleop",state),
                          ("collector",collector_state),("overall",overall),
                          ("samples",samples)]:
            self._set_badge(key,value)
        if collector_missing:
            instruction = "FT Collector diagnostics 대기 중"
            ready = False
        elif collecting:
            instruction = (
                "수집 중: CURRENT → SLOW 무접촉 동작, 접촉 전에 STOP FT EPISODE")
            ready = True
        elif zero_ready:
            instruction = "Zero Gate VERIFIED: START FT EPISODE를 누르세요"
            ready = True
        else:
            instruction = f"START FT EPISODE BLOCKED: {zero_reason}"
            ready = False
        self.next_label.setText(instruction)
        self.next_label.setStyleSheet(
            "padding:8px;background:%s;color:white" %
            ("#16784a" if ready else "#8a3c30"))
        self._refresh_joints(teleop); self._refresh_poses(); self._refresh_plots()
        self._refresh_health(collector)
        busy = self.pipeline_process is not None
        offline_safe = not collecting and state in (
            "IDLE", "PAUSED", "SHUTDOWN", "WAITING")
        self.validate_button.setEnabled(not busy and offline_safe)
        self.train_button.setEnabled(
            not busy and offline_safe and self.validated_dataset is not None)
        self._check_timeouts()
        _events, sequence = self.event_store.snapshot()
        if sequence != self.last_event_sequence:
            self.last_event_sequence = sequence
            self._rebuild_logs()

    def _check_timeouts(self):
        now = time.monotonic()
        if now-self.started_monotonic < 5.0:
            return
        if not self.node.teleop_received or now-self.node.teleop_received > 2.0:
            self._event("ERROR", "Teleop", "Leader teleop status timeout/process offline", dedup_sec=10.0)
        if (not self.node.collector_received
                or now-self.node.collector_received > 2.5):
            self._event(
                "ERROR", "Collector",
                "FT collector diagnostics timeout/process offline",
                dedup_sec=10.0)

    def _set_badge(self,key,value):
        good = str(value) in (
            "FAST", "READY", "RECORDING", "OK", "VERIFIED", "LATCHED")
        bad = (
            "ERROR" in str(value)
            or str(value) in (
                "SHUTDOWN", "OFFLINE",
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

    def _refresh_health(self, collector):
        details = dict(collector)
        details["gui_rss"] = _fmt_bytes(_rss_bytes())
        self.collector_details.setPlainText(
            json.dumps(details, ensure_ascii=False, indent=2, sort_keys=True))

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
        node = holder.get("node")
        episode = "ACTIVE" if node and _collector_is_collecting(node.collector) else "—"
        store.add(severity, source,
                  node.teleop.get("state") if node else "—",
                  episode, message, dedup_sec=dedup_sec)
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
