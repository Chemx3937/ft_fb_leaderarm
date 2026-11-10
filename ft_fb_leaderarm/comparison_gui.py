"""Three-model experiment monitor based on fb_leaderarm's Qt monitor patterns.

See THIRD_PARTY_NOTICES.md. No motor, teleop, bias, or canonical observer outputs.
"""

import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

from .comparison import (ComparisonEngine, ExternalLoadReference, REFERENCE_TOPIC, ROBOT_TOPIC, WRIST_TOPIC,
                         TRIAL_TOPICS, analyze_noise, analyze_trial, averaged_reference_force,
                         load_comparison, replay_frames)
from .contract import SAMPLE_HZ
from .comparison_trace import LIVE_RAMP_PROTOCOL, LiveTraceWriter


COLORS = ("#e9eef5", "#4aa3ff", "#d291ff", "#ffb454")
KINDS = (("free", "60초 무접촉"), ("weak", "약한 접촉 ≤3 N"),
         ("normal", "일반 접촉 >3 N"), ("ramp", "점증 가압 (공통 10회)"),
         ("noise", "10초 정지 센서 노이즈"))


def label(name):
    return "TCN-GRU" if "hybrid" in name else "Phys-MLP" if "phys_ms" in name else "기존 모델"


class ForcePlot(QtWidgets.QWidget):
    """Same time/painter structure as learned_wrench_monitor_gui; four curves."""
    def __init__(self, axis, names):
        super().__init__()
        self.axis, self.names, self.rows = axis, names, []
        self.setMinimumHeight(120)

    def set_rows(self, rows):
        self.rows = rows
        self.update()

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.fillRect(self.rect(), QtGui.QColor("#151b25"))
        p.setPen(QtGui.QColor("#e9eef5"))
        p.drawText(10, 20, f"F{'xyz'[self.axis]} [N] · 손목 실측 + 세 모델 예측 · 최근 20초")
        box = self.rect().adjusted(65, 35, -18, -25)
        p.setPen(QtGui.QColor("#394451")); p.drawRect(box)
        if not self.rows:
            p.drawText(box, QtCore.Qt.AlignCenter, "입력 대기"); return
        end = self.rows[-1]["t"]
        rows = [r for r in self.rows if r["t"] >= end - 20]
        t = np.array([r["t"] - end for r in rows])
        y = np.full((len(rows), 4), np.nan)
        for i, row in enumerate(rows):
            if row["measured"] is not None: y[i, 0] = row["measured"][self.axis]
            for j, name in enumerate(self.names, 1):
                if name in row["predictions"]: y[i, j] = row["predictions"][name][self.axis]
        finite = y[np.isfinite(y)]
        low, high = (min(0., float(finite.min())), max(0., float(finite.max()))) if len(finite) else (-1., 1.)
        margin = max(.15, .08 * (high - low)); low -= margin; high += margin
        p.setPen(QtGui.QColor("#b6c3d2"))
        p.drawText(3, box.top() + 12, f"{high:.2f}"); p.drawText(3, box.bottom(), f"{low:.2f}")
        p.drawText(box.left(), self.height() - 6, "−20 s"); p.drawText(box.right() - 30, self.height() - 6, "now")
        # Keep each curve's extrema per display bucket; never decimate raw bags.
        keep = {0, len(rows) - 1}
        for bucket in np.array_split(np.arange(len(rows)), min(25, len(rows))):
            for col in range(4):
                good = bucket[np.isfinite(y[bucket, col])]
                if len(good):
                    keep.add(int(good[np.argmin(y[good, col])]))
                    keep.add(int(good[np.argmax(y[good, col])]))
            # Preserve missing-data breaks rather than joining across stale input.
            invalid = bucket[~np.isfinite(y[bucket]).all(axis=1)]
            if len(invalid): keep.add(int(invalid[0])); keep.add(int(invalid[-1]))
        indices = sorted(keep)
        for col, color in enumerate(COLORS):
            p.setPen(QtGui.QPen(QtGui.QColor(color), 1.5)); path = QtGui.QPainterPath(); active = False
            for i in indices:
                value = y[i, col]
                if not np.isfinite(value): active = False; continue
                x = box.left() + (t[i] + 20) / 20 * box.width()
                py = box.bottom() - (value - low) / (high - low) * box.height()
                if active: path.lineTo(x, py)
                else: path.moveTo(x, py); active = True
            p.drawPath(path)


def make_ros_node(engine):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from contact_observer_msgs.msg import ObserverInput
    from geometry_msgs.msg import WrenchStamped
    from std_msgs.msg import String
    from .contract import stamp_to_seconds, wrench_from_message

    class Monitor(Node):
        def __init__(self):
            super().__init__("three_model_comparison_monitor")
            self.engine, self.lock, self.latest, self.teleop = engine, threading.Lock(), {}, ("WAITING", 0.)
            self.trace = None
            for topic, typ in [(ROBOT_TOPIC, ObserverInput), (WRIST_TOPIC, WrenchStamped), (REFERENCE_TOPIC, WrenchStamped)]:
                self.create_subscription(typ, topic, lambda m, t=topic: self.receive(t, m), qos_profile_sensor_data)
            self.create_subscription(String, "/leader_teleop_node/status", self.teleop_cb, 10)
            self.create_timer(1 / SAMPLE_HZ, self.tick)

        def now_s(self): return self.get_clock().now().nanoseconds * 1e-9

        def receive(self, topic, msg):
            sample = {"stamp": stamp_to_seconds(msg.header.stamp), "received": self.now_s(), "frame": msg.header.frame_id}
            if topic == ROBOT_TOPIC:
                sample.update(q=np.array(msg.q_rad), dq=np.array(msg.dq_rad_s), sequence=int(msg.source_sequence), valid=bool(msg.valid))
            else: sample["wrench"] = wrench_from_message(msg)
            self.latest[topic] = sample

        def teleop_cb(self, msg):
            try: self.teleop = (str(json.loads(msg.data)["state"]), time.monotonic())
            except (ValueError, KeyError, TypeError): self.teleop = ("INVALID", 0.)

        def tick(self):
            with self.lock:
                began = time.perf_counter()
                inputs = {'robot':self.latest.get(ROBOT_TOPIC), 'wrist':self.latest.get(WRIST_TOPIC), 'external':self.latest.get(REFERENCE_TOPIC)}
                row = self.engine.step(self.now_s(), inputs['robot'], inputs['wrist'], inputs['external'])
                if self.trace:
                    self.trace.submit(row, inputs, self.now_s(), time.perf_counter()-began)

        def start_trace(self, directory):
            with self.lock:
                if self.trace is not None: raise RuntimeError('a live trace is already active')
                self.trace = LiveTraceWriter(directory, self.engine)

        def trace_phase(self, phase):
            with self.lock:
                if self.trace: self.trace.phase = phase

        def finish_trace(self):
            with self.lock:
                trace, self.trace = self.trace, None
            return trace.close() if trace else None

    return Monitor()


def spin_monitor(node):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


class ComparisonWindow(QtWidgets.QMainWindow):
    def __init__(self, engine, session_path, node=None):
        super().__init__()
        self.engine, self.node, self.session_path = engine, node, Path(session_path).resolve()
        self.session = json.loads(self.session_path.read_text())
        ramp_plan = self.session.get('ramp_trial', {})
        self.live_ramp = ramp_plan.get('mode') == LIVE_RAMP_PROTOCOL
        self.ramp_total = int(ramp_plan.get('physical_shared_trials',10))
        self.unloaded_bound = float(ramp_plan.get('unloaded_force_max_n',1.55))
        if self.ramp_total <= 0 or not math.isfinite(self.unloaded_bound) or self.unloaded_bound <= 0:
            raise ValueError('invalid ramp repeat count or unloaded guidance range')
        self.unloaded_since = None
        free_plan = self.session.get("free_trial", {})
        self.free_total = int(free_plan.get("repeats_planned", 20))
        self.free_per_zero = int(free_plan.get("trials_per_zero_session", 5))
        contact_plan = self.session.get("contact_trial", {})
        self.contact_limits = {kind: int(contact_plan.get(f"{kind}_shared_trials", 10)) for kind in ("weak", "normal")}
        weak = contact_plan.get("weak_guidance", {})
        self.weak_min, self.weak_max = float(weak.get("force_min_n", 0.)), float(weak.get("force_max_n", 3.))
        self.weak_average_s = float(weak.get("average_window_s", 0.))
        if not all(math.isfinite(x) for x in (self.weak_min, self.weak_max, self.weak_average_s)) or not (0 <= self.weak_min < self.weak_max and 0 <= self.weak_average_s <= 1.):
            raise ValueError("invalid weak contact guidance range or averaging window")
        upper = self.session.get("ramp_trial", {}).get("force_upper_n")
        self.ramp_upper_n = None if upper is None else float(upper)
        if self.ramp_upper_n is not None and (not math.isfinite(self.ramp_upper_n) or self.ramp_upper_n <= 0):
            raise ValueError("ramp force_upper_n must be finite and positive")
        if self.live_ramp and self.ramp_upper_n is None: raise ValueError('live ramp needs a force upper limit')
        normal = self.session.get("normal_contact_target_n")
        self.normal_target = None if normal is None else float(normal)
        if self.normal_target is not None and (not math.isfinite(self.normal_target) or self.normal_target <= 3 or self.ramp_upper_n is not None and self.normal_target >= self.ramp_upper_n):
            raise ValueError("normal force target must be above 3N and below the upper force")
        self.external_reference = ExternalLoadReference(self.session.get("external_contact_reference_protocol", {}))
        self.reference_protocol = self.external_reference.protocol
        self.process = self.analysis = None
        self.active = None
        self.guide_stage = None
        self.guide_started = 0.
        self.guide_blocked = False
        self.guide_seen_s = 0.
        self.exit_requested = False
        self.setWindowTitle("세 모델 비교 · 진단 전용 · 로봇 힘 feedback OFF")
        self.resize(1450, 1000)
        root = QtWidgets.QWidget(); layout = QtWidgets.QVBoxLayout(root); self.setCentralWidget(root)
        title = QtWidgets.QLabel("세 모델 실시간 비교 — 손목 F/T 좌표계 [aft_sensor2]")
        title.setStyleSheet("font-size:21px;font-weight:bold"); layout.addWidget(title)
        self.health = QtWidgets.QLabel(); layout.addWidget(self.health)
        self.reference = QtWidgets.QLabel(); self.reference.setMinimumHeight(38); layout.addWidget(self.reference)
        self.reference_status = QtWidgets.QLabel(); self.reference_status.setWordWrap(True); layout.addWidget(self.reference_status)
        self.noise = QtWidgets.QLabel(); self.noise.setWordWrap(True); layout.addWidget(self.noise)
        guide = QtWidgets.QHBoxLayout(); layout.addLayout(guide)
        timing = QtWidgets.QVBoxLayout(); guide.addLayout(timing, 3)
        self.phase = QtWidgets.QLabel("기록 시작 전 · 실험 종류를 선택하세요")
        self.phase.setWordWrap(True)
        self.phase.setStyleSheet("font-size:22px;font-weight:bold;padding:6px;background:#34465d")
        timing.addWidget(self.phase)
        self.countdown = QtWidgets.QProgressBar(); self.countdown.setRange(0, 1000)
        self.countdown.setValue(0); self.countdown.setFormat("기록 대기"); timing.addWidget(self.countdown)
        automatic = QtWidgets.QLabel("점증 가압·해제 · 실제 무접촉은 직접 확인 · 실시간 판정은 계산 주기마다 저장" if self.live_ramp else "유지·해제 확인은 외부 하중 기준 · 세 모델 판정과 독립 · 로봇은 직접 조작")
        automatic.setWordWrap(True); timing.addWidget(automatic)
        force = QtWidgets.QVBoxLayout(); guide.addLayout(force, 2)
        self.force_value = QtWidgets.QLabel("외부 힘 — N")
        self.force_value.setStyleSheet("font-size:30px;font-weight:bold;padding:6px")
        self.guide_force_value = QtWidgets.QLabel(); self.guide_force_value.setWordWrap(True)
        self.guide_force_value.setStyleSheet("font-size:20px;font-weight:bold")
        self.force_target = QtWidgets.QLabel(); self.force_target.setWordWrap(True)
        self.force_target.setStyleSheet("font-size:16px;font-weight:bold")
        self.force_bar = QtWidgets.QProgressBar(); self.force_bar.setRange(0, 10000)
        for widget in (self.force_value, self.guide_force_value, self.force_target, self.force_bar): force.addWidget(widget)
        legend = QtWidgets.QHBoxLayout()
        for name, color in zip(["손목 실측"] + [label(n) for n in engine.names], COLORS):
            card = QtWidgets.QLabel(name); card.setStyleSheet(f"color:{color};font-weight:bold"); legend.addWidget(card)
        layout.addLayout(legend)
        self.table = QtWidgets.QTableWidget(4, 6)
        self.table.setHorizontalHeaderLabels(["신호", "Fx [N]", "Fy [N]", "Fz [N]", "잔차 |F| [N]", "현재 판정"])
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers); self.table.setFixedHeight(165); layout.addWidget(self.table)
        self.plots = [ForcePlot(i, engine.names) for i in range(3)]
        for plot in self.plots: layout.addWidget(plot)
        controls = QtWidgets.QHBoxLayout(); self.kind = QtWidgets.QComboBox()
        for key, text in KINDS:
            if key not in self.session.get('enabled_experiments',[k for k,_ in KINDS]): continue
            if key == 'ramp': text = f'점증 가압·해제 (공통 {self.ramp_total}회)'
            if key == "weak" and self.weak_average_s:
                text = f"약한 접촉 · 외부 약 3 N ({self.weak_min:g}–{self.weak_max:g} N 안내)"
            if key == "free": text += f" (총 {self.free_total}회)"
            elif key in self.contact_limits: text += f" (공통 {self.contact_limits[key]}회)"
            self.kind.addItem(text, key)
        self.owner = QtWidgets.QComboBox()
        self.owner.addItem("공통 기록 (세 모델)", "shared")
        self.owner.setEnabled(False)
        self.start = QtWidgets.QPushButton("기록 시작"); self.stop = QtWidgets.QPushButton("기록 종료")
        self.start.clicked.connect(self.start_trial); self.stop.clicked.connect(self.stop_trial)
        for widget in [self.kind, self.owner, self.start, self.stop]: controls.addWidget(widget)
        layout.addLayout(controls)
        self.results = QtWidgets.QLabel("실시간 잔차·판정을 저장하고, 종료 후 기준 준수와 검출 순간의 외부 하중을 계산합니다." if self.live_ramp else "점증 가압의 검출 시각·외부 힘은 원본 bag을 종료 후 오프라인 재생해 계산합니다.")
        self.results.setWordWrap(True); layout.addWidget(self.results)
        self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self.refresh); self.timer.start(200)
        self.setStyleSheet("QWidget {background:#202733;color:#e9eef5} QPushButton {padding:7px;background:#34465d} QPushButton:disabled {background:#28323f;color:#8995a3} QHeaderView::section {background:#34465d}")

    def rows(self):
        if self.node:
            with self.node.lock: return list(self.engine.frames)
        return list(self.engine.frames)

    def refresh(self):
        rows = self.rows(); row = rows[-1] if rows else {"measured": None, "predictions": {}, "states": {}, "residual_norms": {}}
        stale = self.node is not None and (not rows or time.time() - row["t"] > .05)
        teleop = self.node.teleop[0] if self.node and time.monotonic() - self.node.teleop[1] < 2 else "WAITING"
        mode = "LIVE 진단" if self.node else "저장 기록 재생 (기록 버튼 비활성)"
        self.health.setText(f"{mode} · 추론 목표 {SAMPLE_HZ:g} Hz · 모델 준비 32 samples · 손목 잔차 판정 ON/OFF 3.0/1.2 N, hold 20/20 ms · Teleop {teleop}")
        trace = getattr(self.node,'trace',None)
        if self.live_ramp and trace:
            self.health.setText(self.health.text()+f' · 실시간 로그 {trace.submitted} tick · 누락 {trace.dropped}')
        for i, name in enumerate([None] + self.engine.names):
            value = row.get("measured") if name is None else row["predictions"].get(name)
            state = "실측" if name is None else row["states"].get(name, "WAITING")
            if stale: value, state = None, "STALE"
            values = ["손목 실측" if name is None else label(name)] + ([f"{x:+.3f}" for x in value[:3]] if value is not None else ["—"] * 3)
            norm = row["residual_norms"].get(name)
            values += [f"{norm:.3f}" if norm is not None and not stale else "—", state]
            for col, text in enumerate(values):
                cell = QtWidgets.QTableWidgetItem(text)
                cell.setForeground(QtGui.QColor("#ff6464" if state == "CONTACT" else COLORS[i])); self.table.setItem(i, col, cell)
        ref = row.get("reference_force_n") if not stale else None
        if self.live_ramp:
            self.show_reference_state()
        elif not self.active or self.kind.currentData() not in ("weak", "normal", "ramp"):
            self.update_reference_state(rows, ref)
        band = row.get("reference_band", "UNAVAILABLE") if ref is not None else "UNAVAILABLE"
        raw = row.get("reference_wrench")
        components = " / ".join(f"{v:+.3f}" for v in raw[:3]) if raw is not None and ref is not None else "—"
        self.reference.setText(f"외부 F/T [aft_sensor1] 실측 Fx/Fy/Fz: {components} N   |   기준선 보정 |F|: {ref:.3f} N   ·   측정값 기준 {band}" if ref is not None else "외부 F/T: 입력 또는 이 zero-set의 정지 기준선 없음 — 3 N 구분 불가")
        color = '#6c4029' if band == '> 3 N' else '#254756'
        if self.live_ramp:
            self.reference.setText(f"외부 하중계 [aft_sensor1] Fx/Fy/Fz: {components} N  |  기준선 보정 |F|: {ref:.3f} N" if ref is not None else "외부 하중계: 입력·현재 zero-set 기준선을 확인하세요")
            color = '#254756'
        if self.kind.currentData() == "ramp" and self.ramp_upper_n is not None and not self.live_ramp:
            text = f"\n점증 상한 {self.ramp_upper_n:g} N (수동 해제)"
            if ref is not None and ref >= self.ramp_upper_n:
                text += " · 도달: 더 가압하지 말고 해제하세요"; color = '#8f2830'
            self.reference.setText(self.reference.text() + text)
        if self.live_ramp and ref is not None and ref >= self.ramp_upper_n:
            color = '#8f2830'
        self.reference.setStyleSheet(f"padding:7px;background:{color};font-size:16px;font-weight:bold")
        guide_ref = self.guidance_force(rows, ref)
        self.show_force(ref, guide_ref)
        if self.engine.noise:
            parts = []
            for num, text in [("2", "손목"), ("1", "외부")]:
                d = self.engine.noise["sensors"][num]
                parts.append(f"{text} σxyz={np.round(d['force_axis_std_n'],3).tolist()} N, zero 중앙값 크기={d['force_median_norm_n']:.3f} N, centered p99={d['centered_force_norm_p99_p99_9_max_n'][0]:.3f} N")
            self.noise.setText("저장된 정지 노이즈 (절대 정확도 아님), zero=" + self.session["zero_set_id"] + ": " + " | ".join(parts))
        else: self.noise.setText("이 zero-set에 유효한 정지 노이즈 기록이 없습니다. 정지·무접촉 노이즈를 먼저 기록하세요.")
        for plot in self.plots: plot.set_rows(rows)
        kind = self.kind.currentData()
        self.start.setEnabled(self.node is not None and not self.active and self.analysis is None)
        self.stop.setEnabled(self.active is not None)
        self.kind.setEnabled(not self.active)
        if self.exit_requested:
            self.stop_trial()
            if not self.active and self.analysis is None: self.close()
            return
        if self.active:
            elapsed = time.monotonic() - self.active["monotonic"]
            if "stop_s" in self.active:
                self.phase.setText("기록 종료 중 · 원본을 저장하고 있습니다"); return
            if kind in ("weak", "normal", "ramp"):
                previous_stage = self.guide_stage
                if kind == 'ramp' and self.live_ramp:
                    self.refresh_live_ramp_guide(ref, rows)
                    trace = getattr(self.node,'trace',None)
                    if trace and (trace.error or trace.dropped):
                        self.results.setText('실시간 로그 오류 · 직접 가압 중단·해제하세요 · 원본은 보존합니다')
                        self.stop_trial()
                else: self.refresh_contact_guide(ref, rows, guide_ref)
                if self.guide_stage != previous_stage: self.show_force(ref, guide_ref)
            else:
                if elapsed < 2: cue, remaining, duration = "준비 — 정지 유지", 2-elapsed, 2.
                elif kind == "free":
                    t = elapsed - 2
                    cue, end, duration = ("정지",10,10) if t<10 else ("저속 왕복",30,20) if t<30 else ("정상 속도 왕복",50,20) if t<50 else ("정지",60,10)
                    remaining = max(0.,end-t)
                else: cue, remaining, duration = "정지·공중 무접촉 유지", max(0.,12-elapsed), 10.
                self.phase.setText(f"{self.active['number']}회차 · 전체 {elapsed:.1f}초\n{cue}")
                self.countdown.setValue(int(1000*(1-remaining/duration)))
                self.countdown.setFormat(f"남은 {remaining:.1f}초 / {duration:g}초")
            limit = 62 if kind == "free" else 12 if kind == "noise" else 120
            if elapsed >= limit: self.stop_trial()

    def guidance_force(self, rows, ref):
        if ref is None: return None
        if self.kind.currentData() == "weak" and self.weak_average_s:
            return averaged_reference_force(rows, self.engine.reference_baseline, self.weak_average_s)
        return ref

    def force_in_guide_band(self, value):
        return value is not None and (self.weak_min <= value <= self.weak_max if self.kind.currentData() == "weak" else value > 3.)

    def show_force(self, ref, guide_ref):
        kind = self.kind.currentData(); color = "#b6c3d2"
        averaging = kind == "weak" and self.weak_average_s > 0
        self.guide_force_value.setVisible(averaging)
        self.guide_force_value.setText(f"원본 외부 힘 {ref:.2f} N · {self.weak_average_s*1000:g} ms 평균 안내" if ref is not None else "원본 외부 힘: 입력 없음")
        if ref is None:
            self.force_value.setText("외부 힘 — N"); self.force_target.setText("유효한 외부 센서와 정지 기준선 필요")
            self.force_bar.setValue(0); self.force_bar.setFormat("입력 없음")
        else:
            self.force_value.setText(f"외부 힘 {ref:.2f} N")
            if averaging:
                self.force_value.setText(f"안내 힘 {guide_ref:.2f} N" if guide_ref is not None else "안내 힘 — N")
            difference = f"3 N보다 {abs(ref-3):.2f} N {'높음' if ref>3 else '낮음'}" if ref != 3 else "3 N과 같음"
            text = f"{'3 N 초과' if ref>3 else '3 N 이하'} · {difference}"
            color = "#ffb454" if ref > 3 else "#4aa3ff"
            if kind == "weak":
                if averaging:
                    text = f"안내 목표 {self.weak_min:g}–{self.weak_max:g} N · "
                    text += "범위 내 — 유지하세요" if self.force_in_guide_band(guide_ref) else "평균 입력 대기" if guide_ref is None else "조금 더 가압" if guide_ref < self.weak_min else "힘을 줄이세요"
                    color = "#65d7a0" if self.force_in_guide_band(guide_ref) else "#ffb454" if guide_ref is None or guide_ref < self.weak_min else "#ff6464"
                else:
                    text = "약한 접촉 목표 ≤3 N · " + difference
                    if ref > 3: text = "3 N 초과 — 힘을 줄이세요 · " + difference; color = "#ff6464"
            elif kind == "normal":
                target = self.normal_target
                text = (f"일반 목표 약 {target:g} N (>3 N) · " if target is not None else "일반 접촉 목표 >3 N · ") + difference
                color = "#65d7a0" if ref > 3 else "#ffb454"
            elif kind == "ramp" and self.ramp_upper_n is not None:
                text = f"상한 {self.ramp_upper_n:g} N · 남은 힘 {max(0.,self.ramp_upper_n-ref):.2f} N"
                if self.live_ramp: color = '#4aa3ff'
                if ref >= self.ramp_upper_n: text = "상한 도달 — 가압 중단 · 완전히 해제하세요"; color = "#ff6464"
            if kind in ("weak","normal","ramp"):
                stage = self.guide_stage if self.active else None
                cues = {None:"기록 시작 전 무접촉 유지", "prepare":"준비 중 · 무접촉 유지",
                        "pre_free":"무접촉 유지 · 접근 안내를 기다리세요",
                        "release":"가압 중단 · 완전히 해제하세요 · 자동 확인",
                        "post_free":"무접촉 유지 · 3초를 자동으로 셉니다", "done":"완료 · 기록 종료를 누르세요"}
                if stage in cues:
                    text, color = cues[stage], "#ffb454" if stage == "release" else "#4aa3ff"
                    if self.live_ramp and stage == 'release': text = '가압 중단 · 서서히 힘을 줄여 완전히 해제하세요'
                    if kind == "normal" and self.normal_target is not None and stage in (None,"prepare","pre_free"):
                        text += f" · 일반 목표 약 {self.normal_target:g} N (>3 N)"
            if self.ramp_upper_n is not None and ref >= self.ramp_upper_n:
                text = f"외부 힘 {self.ramp_upper_n:g} N 도달 — 가압 중단 · 해제하세요"; color = "#ff6464"
                self.force_value.setText(f"원본 외부 힘 {ref:.2f} N")
            if self.live_ramp and self.active and 'stop_s' in self.active:
                text,color='기록 종료 중 · 가압 중단·완전 해제하세요','#ffb454'
            self.force_target.setText(text)
            scale = self.ramp_upper_n or 10.
            bar = guide_ref if averaging and guide_ref is not None and ref < scale else ref
            self.force_bar.setValue(min(10000, int(bar/scale*10000)))
            self.force_bar.setFormat(f"{bar:.2f} N / 눈금 {scale:g} N")
        self.force_value.setStyleSheet(f"font-size:30px;font-weight:bold;padding:6px;color:{color}")
        self.force_bar.setStyleSheet(f"QProgressBar::chunk {{background:{color}}}")

    def set_guide_stage(self, stage):
        self.guide_stage, self.guide_started = stage, time.monotonic()
        self.guide_blocked = False
        self.guide_seen_s = time.time()
        if stage == "prepare":
            if not self.live_ramp: self.external_reference.reset(self.guide_seen_s)
            self.unloaded_since = None
        if self.live_ramp and self.node and hasattr(self.node,'trace_phase'):
            self.node.trace_phase(stage)
        self.active["guide_events"].append({"stage": stage, "t_s": time.time()})
        if stage == "done": self.active["guide_complete"] = True

    def update_reference_state(self, rows, ref):
        states = []
        reference = self.external_reference
        if rows and reference.seen_s is not None and rows[-1]["t"] < reference.seen_s:
            reference.update(rows[-1]["t"], None)
            self.show_reference_state()
            return [None]
        for row in rows:
            stamp = row["t"]
            if reference.seen_s is not None and stamp <= reference.seen_s: continue
            states.append(reference.update(stamp, row.get("reference_force_n")))
        if ref is None:
            reference.invalidate(); states.append(None)
        self.show_reference_state()
        return states

    def show_reference_state(self):
        if self.live_ramp:
            self.reference_status.setText('접촉 판정: 각 손목 observer · 외부 센서: 검출 순간 하중 측정용 · 외부 CONTACT 문턱으로 검출을 거르지 않습니다')
            return
        state = self.external_reference.state
        text = "CONTACT" if state is True else "FREE" if state is False else "UNKNOWN"
        self.reference_status.setText(f"외부 하중 기준: {text} · ON/OFF {self.reference_protocol['force_on_n']:g}/{self.reference_protocol['force_off_n']:g} N"
                                      " · 외부 약 3 N은 조작 목표이며 손목 잔차 3 N과 같은 검출 경계가 아닙니다")

    def refresh_live_ramp_guide(self, ref, rows):
        stage = self.guide_stage
        recent = [r for r in rows if r['t'] > self.guide_seen_s]
        if rows: self.guide_seen_s = rows[-1]['t']
        valid = ref is not None and bool(rows) and rows[-1]['measured'] is not None
        valid = valid and all(r.get('reference_force_n') is not None and r['measured'] is not None for r in recent)
        unloaded = valid and ref <= self.unloaded_bound and all(r['reference_force_n'] <= self.unloaded_bound for r in recent)
        duration = {'prepare':2.,'pre_free':3.,'post_free':3.}.get(stage)
        if duration and (not valid or stage == 'post_free' and not unloaded): self.guide_started = time.monotonic()
        elapsed = time.monotonic()-self.guide_started
        if stage == 'release':
            if not unloaded: self.unloaded_since = None
            elif self.unloaded_since is None: self.unloaded_since = time.monotonic()
            elif time.monotonic()-self.unloaded_since >= .2:
                self.set_guide_stage('post_free'); return self.refresh_live_ramp_guide(ref,rows)
        if duration and elapsed >= duration:
            self.set_guide_stage({'prepare':'pre_free','pre_free':'approach','post_free':'done'}[stage])
            return self.refresh_live_ramp_guide(ref,rows)
        if stage == 'approach' and ((ref is not None and ref >= self.ramp_upper_n) or any(r.get('reference_force_n') is not None and r['reference_force_n'] >= self.ramp_upper_n for r in recent)):
            self.set_guide_stage('release'); return self.refresh_live_ramp_guide(ref,rows)
        cues={'prepare':'준비 · 정지 유지', 'pre_free':'① 실제 무접촉 3초 · 아직 접근하지 마세요',
              'approach':f'② 같은 방향으로 천천히 가압 · {self.ramp_upper_n:g} N에서 가압 중단',
              'release':'③ 서서히 힘을 줄여 완전히 해제 · 무부하 복귀 안내 대기',
              'post_free':'④ 실제 무접촉 3초 · 센서에서 완전히 떨어져 유지', 'done':'완료 · [기록 종료]를 누르세요'}
        cue = cues[stage] if valid else '센서 입력 확인 중 · 가압 중단·해제하고 입력을 확인하세요'
        self.phase.setText(f"{self.active['number']}/{self.ramp_total}회차 · 전체 {time.monotonic()-self.active['monotonic']:.1f}초\n{cue}")
        self.countdown.setValue(int(1000*min(1.,elapsed/duration)) if duration else 1000 if stage=='done' else 0)
        self.countdown.setFormat(f'남은 {max(0.,duration-elapsed):.1f}초 / {duration:g}초' if duration else '기록 종료 가능' if stage=='done' else '하중 측정 · 로봇 직접 조작')

    def refresh_contact_guide(self, ref, rows, guide_ref):
        stage = self.guide_stage
        reference_states = self.update_reference_state(rows, ref)
        duration = {"prepare":2., "pre_free":3., "hold":2., "post_free":3.}.get(stage)
        in_band = self.force_in_guide_band(guide_ref)
        recent = [r for r in rows if r["t"] > self.guide_seen_s]
        if rows: self.guide_seen_s = rows[-1]["t"]
        # Guide stages use external load only; model decisions remain evaluation outputs.
        target_reached = self.active["kind"] != "normal" or self.normal_target is None or (ref is not None and ref >= self.normal_target)
        if stage == "approach" and self.active["kind"] in ("weak","normal") and self.external_reference.state is True and in_band and target_reached:
            self.set_guide_stage("hold")
            return self.refresh_contact_guide(ref, rows, guide_ref)
        if stage == "release" and self.external_reference.state is False:
            self.set_guide_stage("post_free")
            return self.refresh_contact_guide(ref, rows, guide_ref)
        invalid = (stage in ("pre_free","post_free") and (self.external_reference.state is not False or any(s is not False for s in reference_states))) or (
                   stage == "hold" and (self.external_reference.state is not True or any(s is not True for s in reference_states) or not in_band))
        if stage == "hold" and not (self.active["kind"] == "weak" and self.weak_average_s):
            invalid = invalid or any(r.get("reference_force_n") is None or
                (r["reference_force_n"] > 3 if self.active["kind"] == "weak" else r["reference_force_n"] <= 3) for r in recent)
        if invalid or self.guide_blocked: self.guide_started = time.monotonic()
        self.guide_blocked = invalid
        elapsed = time.monotonic() - self.guide_started
        if duration is not None and elapsed >= duration:
            self.set_guide_stage({"prepare":"pre_free", "pre_free":"approach", "hold":"release", "post_free":"done"}[stage])
            return self.refresh_contact_guide(ref, rows, guide_ref)
        if stage == "approach" and self.active["kind"] == "ramp" and self.ramp_upper_n is not None:
            if (ref is not None and ref >= self.ramp_upper_n) or any(r.get("reference_force_n") is not None and r["reference_force_n"] >= self.ramp_upper_n for r in recent):
                self.set_guide_stage("release")
                return self.refresh_contact_guide(ref, rows, guide_ref)
        cue = {"prepare":"준비 · 정지 유지", "pre_free":"① 무접촉 유지 · 아직 접근하지 마세요",
               "approach":"② 천천히 접근 · 목표 힘 도달 시 유지시간 자동 시작",
               "hold":"③ 접촉 유지 · 목표 힘을 유지하세요", "release":"④ 가압 중단 · 완전히 해제하세요 · 외부 F/T로 자동 확인",
               "post_free":"⑤ 무접촉 유지 · 센서에서 떨어져 있으세요", "done":"완료 · [기록 종료]를 누르세요"}[stage]
        if stage == "approach" and self.active["kind"] == "ramp":
            cue = f"② 천천히 점증 가압 · {self.ramp_upper_n:g} N에서 가압 중단" if self.ramp_upper_n is not None else "② 천천히 점증 가압 · 정한 종료 힘에서 해제"
        if stage == "release" and self.active["kind"] == "ramp": cue = "③ 상한 도달 · 가압 중단 · 완전히 해제하세요 · 자동 확인"
        if invalid:
            if self.external_reference.state is None: cue = "외부 센서 입력·하중 상태 확인 중 · 카운트 대기"
            elif stage in ("pre_free","post_free"): cue = "외부 하중·재접촉 감지 · 무접촉 확인 후 3초를 다시 셉니다"
            else: cue = "목표 힘·접촉 유지 확인 중 · 조건을 맞추면 2초를 다시 셉니다"
        total = time.monotonic()-self.active["monotonic"]
        self.phase.setText(f"{self.active['number']}회차 · 전체 {total:.1f}초\n{cue}")
        self.countdown.setValue(int(1000*min(1.,elapsed/duration)) if duration else 1000 if stage == "done" else 0)
        self.countdown.setFormat(f"남은 {max(0.,duration-elapsed):.1f}초 / {duration:g}초" if duration else "외부 F/T 자동 확인 중" if stage != "done" else "기록 종료 가능")

    def start_trial(self):
        if self.active or self.analysis is not None: return
        rows = self.rows(); kind = self.kind.currentData()
        if not rows or rows[-1]["measured"] is None or time.time() - rows[-1]["t"] > .05:
            QtWidgets.QMessageBox.warning(self, "입력 준비", "유효한 현재 손목 F/T 입력이 필요합니다."); return
        if kind != "noise" and self.engine.noise is None:
            QtWidgets.QMessageBox.warning(self, "정지 노이즈 확인", "이 zero-set의 10초 정지 센서 노이즈를 먼저 기록하고 PASS를 확인하세요."); return
        if kind != "noise" and any(rows[-1]["states"].get(n) not in ("FREE", "CONTACT") for n in self.engine.names):
            QtWidgets.QMessageBox.warning(self, "기록 준비", "세 모델의 유효한 추론 입력이 필요합니다."); return
        current = json.loads(self.session_path.read_text())
        if current["zero_set_id"] != self.session["zero_set_id"]:
            QtWidgets.QMessageBox.warning(self, "zero-set 변경", "새 zero-set 세션으로 GUI를 다시 실행하세요."); return
        if kind != "noise" and (time.monotonic() - self.node.teleop[1] >= 2 or self.node.teleop[0] != "FAST"):
            QtWidgets.QMessageBox.warning(self, "Teleop 준비", "기존 리더암 터미널에서 FAST 상태를 준비하세요. 이 GUI는 로봇을 움직이지 않습니다."); return
        if kind == "noise":
            st = self.node.latest.get(ROBOT_TOPIC)
            if not ComparisonEngine.fresh(st, time.time(), "right_base_link") or not np.isfinite(st["q"]).all() or not np.isfinite(st["dq"]).all() or np.max(np.abs(st["dq"])) > .02 or np.max(np.abs(np.rad2deg(st["q"]) - [5.5,52,112,28,-107,-35])) > 1:
                QtWidgets.QMessageBox.warning(self, "정지 준비", "기준 자세에서 완전히 정지한 뒤 노이즈를 기록하세요."); return
        if kind != "free" and kind != "noise" and rows[-1].get("reference_force_n") is None:
            QtWidgets.QMessageBox.warning(self, "외부 F/T", "이 zero-set의 외부 센서 기준선과 유효 입력이 필요합니다."); return
        owner = "shared"
        base = self.session_path.parent / "trials"; base.mkdir(exist_ok=True)
        count = per_zero = 0
        exclusions = {item["trial"] for item in current.get("trial_selection", {}).get("excluded_from_planned_counts", [])}
        for p in self.session_path.parent.parent.glob("session_*/trials/*/trial.json"):
            d = json.loads(p.read_text())
            done = d.get("complete", False) and d.get("analysis_complete", False)
            report = p.parent / "comparison_report.json"
            if report.is_file() and d["kind"] in ("weak", "normal", "ramp"):
                report_data = json.loads(report.read_text())
                done = done and report_data.get("protocol_quality", {}).get("plan_eligible", True)
                if self.live_ramp and d['kind']=='ramp': done = done and report_data.get('analysis_schema')==LIVE_RAMP_PROTOCOL
            done = done and str(p.parent.relative_to(self.session_path.parent.parent)) not in exclusions
            count += int(d["kind"] == kind and done)
            per_zero += int(d["kind"] == "free" and d["zero_set_id"] == self.session["zero_set_id"] and done)
        limit = {"free":self.free_total, **self.contact_limits, "ramp":self.ramp_total}.get(kind)
        if limit and count >= limit:
            QtWidgets.QMessageBox.information(self, "계획 횟수 완료", f"{kind}: 계획한 {limit}회를 이미 기록했습니다."); return
        if kind == "free" and per_zero >= self.free_per_zero:
            QtWidgets.QMessageBox.information(self, "독립 zero-set 세션", f"현재 zero-set의 무접촉 {self.free_per_zero}회를 완료했습니다. 다음 zero-set 세션에서 계속합니다. 전체 목표는 {self.free_total}회입니다."); return
        directory = base / (datetime.now().strftime("%Y%m%d_%H%M%S_%f") + "_" + kind)
        directory.mkdir()
        self.active = {"directory": str(directory), "kind": kind, "owner": owner, "number": count+1,
                       "monotonic": time.monotonic(), "start_s": time.time(), "zero_set_id": self.session["zero_set_id"]}
        if kind in ("weak", "normal", "ramp"):
            self.active.update(guide_events=[], guide_complete=False, guide_mode=LIVE_RAMP_PROTOCOL if self.live_ramp and kind=='ramp' else "automatic_external_ft", guide_reference_protocol=None if self.live_ramp else dict(self.reference_protocol))
            self.set_guide_stage("prepare")
        if self.live_ramp and kind == 'ramp':
            try: self.node.start_trace(directory)
            except Exception as exc:
                self.active=None
                QtWidgets.QMessageBox.warning(self,'실시간 로그 준비 실패',str(exc));return
        self.process = QtCore.QProcess(self)
        self.process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        self.process.readyReadStandardOutput.connect(lambda: self.results.setText(bytes(self.process.readAllStandardOutput()).decode(errors="replace")[-700:]))
        self.process.finished.connect(self.recording_finished)
        self.process.errorOccurred.connect(self.recording_error)
        qos = self.session_path.parent / "record_qos.yaml"
        args = ["bag", "record", "-s", "sqlite3", "-o", str(directory / "raw")]
        if qos.is_file(): args += ["--qos-profile-overrides-path", str(qos)]
        self.process.start("ros2", args + list(TRIAL_TOPICS[kind]))
        # Save the fixed session snapshot used later by offline analysis.
        snapshot = json.loads(json.dumps(self.session))
        if self.engine.noise is not None:
            (directory / "noise_baseline.json").write_text(json.dumps(self.engine.noise, indent=2) + "\n")
            snapshot["post_zero_static_diagnostic"] = {"report": "noise_baseline.json"}
        (directory / "session.json").write_text(json.dumps(snapshot, indent=2, ensure_ascii=False) + "\n")

    def stop_trial(self):
        if self.active and "stop_s" not in self.active and self.process and self.process.state() != QtCore.QProcess.NotRunning:
            self.active["stop_s"] = time.time()
            if self.live_ramp and self.active['kind']=='ramp': self.node.trace_phase('stopping')
            pid = int(self.process.processId())
            if pid > 0:
                try: os.kill(pid, signal.SIGINT)
                except ProcessLookupError: pass
            process = self.process
            QtCore.QTimer.singleShot(3000, lambda: process.kill() if process.state() != QtCore.QProcess.NotRunning else None)

    def recording_error(self, error):
        self.results.setText(f"rosbag 실행 오류: {self.process.errorString()}")
        if error == QtCore.QProcess.FailedToStart:
            self.recording_finished(-1, QtCore.QProcess.CrashExit)

    def recording_finished(self, exit_code, exit_status):
        if not self.active: return
        meta = dict(self.active); directory = Path(meta["directory"]); kind = meta["kind"]
        trace_info = self.node.finish_trace() if self.live_ramp and kind == 'ramp' else None
        end = meta.pop("stop_s", time.time()); elapsed = time.monotonic() - meta.pop("monotonic")
        meta["score_start_s"] = meta["start_s"] + 2
        meta["score_end_s"] = min(end, meta["start_s"] + (62 if kind == "free" else 12)) if kind in ("free", "noise") else end
        meta["complete"] = bool((directory / "raw/metadata.yaml").is_file() and exit_code == 0 and elapsed >= (61.9 if kind == "free" else 11.9 if kind == "noise" else 2.1))
        if kind in ("weak", "normal", "ramp"): meta["complete"] = meta["complete"] and meta["guide_complete"]
        if self.live_ramp and kind == 'ramp':
            meta['live_trace_complete'] = bool(trace_info and trace_info['complete'])
            meta['complete'] = meta['complete'] and meta['live_trace_complete']
        meta["raw_topics"] = list(TRIAL_TOPICS[kind]); meta["recorder_exit_code"] = exit_code
        (directory / "trial.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False) + "\n")
        self.active = None; self.phase.setText("기록 종료 · 원본 보존 · 오프라인 계산 중")
        if not meta["complete"]:
            self.results.setText("불완전 기록으로 보존했습니다. 계획 횟수에 포함하지 않습니다."); return
        self.analysis = QtCore.QProcess(self)
        self.analysis.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        self.analysis.readyReadStandardOutput.connect(lambda: self.results.setText(bytes(self.analysis.readAllStandardOutput()).decode(errors="replace")[-1500:]))
        self.analysis.finished.connect(lambda code, status: self.analysis_finished(directory, code))
        self.analysis.errorOccurred.connect(lambda error: self.analysis_finished(directory, -1) if error == QtCore.QProcess.FailedToStart else None)
        self.analysis.start(sys.executable, ["-m", "ft_fb_leaderarm.comparison_gui", "--session", str(directory / "session.json"), "--analyze-trial", str(directory)])

    def analysis_finished(self, directory, code):
        self.analysis = None
        path = directory / "comparison_report.json"
        if code != 0: self.phase.setText("오프라인 분석 실패 · 원본 보존 · 완료 횟수에 포함하지 않음"); return
        self.phase.setText("원본·결과 저장 완료: " + str(directory))
        if path.is_file():
            result = json.loads(path.read_text()); onsets = result.get("ramp_onsets")
            if result.get('analysis_schema') == LIVE_RAMP_PROTOCOL:
                status_text={'NO_NEW_DETECTION':f'{self.ramp_upper_n:g} N까지 새 검출 없음','PREACTIVATED':'가압 전부터 CONTACT',
                             'EXTERNAL_UNPAIRED':'검출됨·외부 힘 시각 대응 불가','DETECTED_AFTER_UPPER':'상한 도달 후 검출',
                             'NO_DETECTION_WITH_INVALID_TICKS':'미검출·입력 유효성 확인 필요','NO_LOADING_PHASE':'가압 구간 없음'}
                verdict_text={'PASS':'관측 tick 기준 준수','INCOMPLETE':'증거 부족·주기 확인','VIOLATION':'기준 불일치'}
                self.results.setText('실시간 검출 외부 하중: '+' | '.join(
                    f"{label(n)}: {v['external_force_n']:.3f} N" if v['status']=='DETECTED' else f"{label(n)}: {status_text.get(v['status'],v['status'])}"
                    for n,v in result['ramp_detection'].items())+'\n판정 기준 검사: '+' | '.join(
                    f"{label(n)} {verdict_text[v['verdict']]}" for n,v in result['detector_conformance']['models'].items()))
                if not result['protocol_quality']['plan_eligible']:
                    self.phase.setText('저장 완료 · 가압·해제 절차 또는 로그 범위 확인 필요 · 계획 횟수 제외')
                return
            if onsets:
                self.results.setText("접촉 기준 구간 내 첫 검출 외부 힘: " + " | ".join(f"{label(k)}: {v['external_force_n']:.3f} N" if v and v.get("reference_valid") else f"{label(k)}: 새 검출 없음/선행 CONTACT/외부 동기 불량" for k,v in onsets.items()))
            hold = result.get("guided_hold_window")
            if hold:
                self.results.setText("최종 안내 유지 2초의 CONTACT 출력 비율: " + " | ".join(
                    f"{label(k)}: {v['contact_fraction']:.0%}" if v['contact_fraction'] is not None else f"{label(k)}: 유효 입력 없음"
                    for k,v in hold['models'].items()) + " · 외부 목표 힘 유지와 모델 검출은 별개입니다")
            quality = result.get("protocol_quality", {})
            if quality and (not quality['single_complete_reference_contact'] or quality['weak_exceeded_normal_target']):
                self.results.setText(self.results.text() + f"\n프로토콜 확인: 외부 접촉 구간 {quality['reference_contact_intervals']}개, 약한 시행에서 일반 목표 초과={quality['weak_exceeded_normal_target']} · 원본 보존")
            if quality.get('plan_eligible') is False:
                self.phase.setText("원본·분석 저장 완료 · 외부 하중 조건 이탈로 계획 횟수에서 제외 · 해제 후 다시 기록하세요")
        else:
            report = json.loads((directory / "noise_report.json").read_text())
            if report["static_zero_gate_pass"]:
                updated = json.loads((directory / "session.json").read_text())
                current = json.loads(self.session_path.read_text())
                if current["zero_set_id"] != self.session["zero_set_id"]:
                    self.results.setText("zero-set이 변경돼 이전 기준선을 적용하지 않았습니다. GUI를 새 세션으로 다시 실행하세요."); return
                current["post_zero_static_diagnostic"] = updated["post_zero_static_diagnostic"]
                self.session = current
                temporary = self.session_path.with_suffix(".tmp.json")
                temporary.write_text(json.dumps(current, indent=2, ensure_ascii=False) + "\n"); temporary.replace(self.session_path)
                if self.node:
                    with self.node.lock:
                        self.engine.noise = report; self.engine.reference_baseline = np.array(report["sensors"]["1"]["force_median_n"])
            self.results.setText("정지 zero 품질: " + ("PASS" if report["static_zero_gate_pass"] else "FAIL — 자세·하중·센서 상태 확인 필요"))

    def request_exit(self):
        self.exit_requested = True
        self.phase.setText("종료 요청 · 현재 기록 저장과 분석을 마친 뒤 닫습니다.")
        self.stop_trial()
        if not self.active and self.analysis is None: self.close()

    def closeEvent(self, event):
        if self.active or self.analysis is not None:
            QtWidgets.QMessageBox.information(self, "기록/분석 중", "기록 종료와 결과 저장을 마친 뒤 창을 닫으세요."); event.ignore()
        else: event.accept()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True)
    parser.add_argument("--replay-bag")
    parser.add_argument("--screenshot")
    parser.add_argument("--analyze-trial")
    args = parser.parse_args(argv)
    import torch
    torch.set_num_threads(1)
    if args.analyze_trial:
        meta = json.loads((Path(args.analyze_trial) / "trial.json").read_text())
        result = analyze_noise(args.analyze_trial, args.session) if meta["kind"] == "noise" else analyze_trial(args.analyze_trial, args.session)
        print(json.dumps(result, indent=2)); return
    manifest, engine, noise = load_comparison(args.session)
    node = thread = None
    if args.replay_bag:
        replay_frames(engine, args.replay_bag)
    else:
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO); node = make_ros_node(engine)
        thread = threading.Thread(target=spin_monitor, args=(node,), daemon=True); thread.start()
    app = QtWidgets.QApplication(sys.argv[:1]); window = ComparisonWindow(engine, args.session, node); window.show()
    previous_handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    for sig in previous_handlers:
        signal.signal(sig, lambda *_: window.request_exit())
    if args.screenshot:
        if not args.replay_bag: raise ValueError("screenshots require offline --replay-bag")
        QtCore.QTimer.singleShot(700, lambda: (window.grab().save(args.screenshot), app.quit()))
    try: return app.exec_()
    finally:
        for sig, handler in previous_handlers.items(): signal.signal(sig, handler)
        if node:
            if node.trace is not None: node.finish_trace()
            if rclpy.ok(): rclpy.shutdown()
            thread.join(timeout=3); node.destroy_node()


if __name__ == "__main__":
    main()
