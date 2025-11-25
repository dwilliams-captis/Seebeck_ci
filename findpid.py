#!/usr/bin/env python3
"""
PID Auto-Tuning for Seebeck Heater Control System
Integrates with MCC134 thermocouples and gpiozero PWM heaters
"""

import sys
import csv
import time
import math
from datetime import datetime
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QComboBox, QProgressBar, QGroupBox, QSpinBox,
    QDoubleSpinBox, QTextEdit, QTabWidget
)
from PyQt5.QtCore import QTimer, Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from gpiozero import PWMOutputDevice
from daqhats import mcc134, hat_list, HatIDs, TcTypes


class TuningMethod(Enum):
    ZN_CLASSIC = "Ziegler-Nichols Classic"
    ZN_NO_OVERSHOOT = "Ziegler-Nichols No Overshoot"
    TYREUS_LUYBEN = "Tyreus-Luyben"
    COHEN_COON = "Cohen-Coon"


@dataclass
class PIDParams:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0


@dataclass
class AutoTuneResult:
    ultimate_gain: float
    oscillation_period: float
    params: dict = field(default_factory=dict)


class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, setpoint=25.0,
                 output_min=0.0, output_max=1.0, sample_time=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.output_min = output_min
        self.output_max = output_max
        self.sample_time = sample_time
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = None
        self._last_input = None
        self._integral_max = output_max / max(ki, 0.001)

    def reset(self):
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = None
        self._last_input = None

    def set_tunings(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self._integral_max = self.output_max / max(ki, 0.001)

    def compute(self, current_value: float, current_time: float = None) -> float:
        if current_time is None:
            current_time = time.time()
        error = self.setpoint - current_value
        dt = self.sample_time if self._last_time is None else current_time - self._last_time
        if dt < 0.1:
            return self._clamp(self.kp * error)

        p_term = self.kp * error
        self._integral += error * dt
        self._integral = max(-self._integral_max, min(self._integral_max, self._integral))
        i_term = self.ki * self._integral
        d_term = -self.kd * ((current_value - self._last_input) / dt) if self._last_input else 0.0

        self._last_error, self._last_time, self._last_input = error, current_time, current_value
        return self._clamp(p_term + i_term + d_term)

    def _clamp(self, value: float) -> float:
        return max(self.output_min, min(self.output_max, value))


class RelayAutoTuner:
    def __init__(self, setpoint: float, output_min=0.0, output_max=1.0,
                 hysteresis=0.5, min_cycles=4, max_cycles=10, timeout=900.0):
        self.setpoint = setpoint
        self.output_min, self.output_max = output_min, output_max
        self.hysteresis = hysteresis
        self.min_cycles, self.max_cycles = min_cycles, max_cycles
        self.timeout = timeout
        self.reset()

    def reset(self):
        self._relay_state = True
        self._peaks, self._valleys, self._zero_crossings = [], [], []
        self._last_value, self._start_time = None, None
        self._cycle_count = 0
        self._increasing = None
        self.data_log = []

    def update(self, current_value: float, current_time: float = None):
        if current_time is None:
            current_time = time.time()
        if self._start_time is None:
            self._start_time = current_time

        if current_time - self._start_time > self.timeout:
            return self.output_min, True, None

        error = self.setpoint - current_value
        if self._relay_state and error < -self.hysteresis:
            self._relay_state = False
        elif not self._relay_state and error > self.hysteresis:
            self._relay_state = True

        output = self.output_max if self._relay_state else self.output_min
        self.data_log.append((current_time - self._start_time, current_value, output, self.setpoint))

        if self._last_value is not None:
            increasing = current_value > self._last_value
            if self._increasing is not None:
                if self._increasing and not increasing:
                    self._peaks.append((current_time, current_value))
                elif not self._increasing and increasing:
                    self._valleys.append((current_time, current_value))
                    self._cycle_count += 1
            if (self._last_value < self.setpoint <= current_value or
                self._last_value > self.setpoint >= current_value):
                self._zero_crossings.append(current_time)
            self._increasing = increasing

        self._last_value = current_value
        if self._cycle_count >= self.min_cycles and len(self._peaks) >= 3:
            result = self._calculate_result()
            if result:
                return output, True, result
        if self._cycle_count >= self.max_cycles:
            return output, True, self._calculate_result()
        return output, False, None

    def _calculate_result(self) -> Optional[AutoTuneResult]:
        if len(self._peaks) < 2 or len(self._valleys) < 2:
            return None
        peak_vals = [p[1] for p in self._peaks[-4:]]
        valley_vals = [v[1] for v in self._valleys[-4:]]
        amplitude = (sum(peak_vals)/len(peak_vals) - sum(valley_vals)/len(valley_vals)) / 2
        if amplitude < 0.01:
            return None

        if len(self._zero_crossings) >= 4:
            periods = [self._zero_crossings[i] - self._zero_crossings[i-2] for i in range(2, len(self._zero_crossings))]
            tu = sum(periods) / len(periods)
        elif len(self._peaks) >= 2:
            periods = [self._peaks[i][0] - self._peaks[i-1][0] for i in range(1, len(self._peaks))]
            tu = sum(periods) / len(periods)
        else:
            return None

        relay_amp = (self.output_max - self.output_min) / 2
        ku = (4 * relay_amp) / (math.pi * amplitude)
        return AutoTuneResult(ku, tu, self._calc_pid_params(ku, tu))

    def _calc_pid_params(self, ku: float, tu: float) -> dict:
        return {
            TuningMethod.ZN_CLASSIC: PIDParams(0.6*ku, 1.2*ku/tu, 0.075*ku*tu),
            TuningMethod.ZN_NO_OVERSHOOT: PIDParams(0.2*ku, 0.4*ku/tu, 0.066*ku*tu),
            TuningMethod.TYREUS_LUYBEN: PIDParams(0.45*ku, 0.54*ku/tu, 0.0),
            TuningMethod.COHEN_COON: PIDParams(0.49*ku, 0.98*ku/tu, 0.06*ku*tu),
        }


class AutoTunerApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PID Auto-Tuner for Heater System")
        self.setFixedSize(900, 650)

        # Hardware init
        self.heater1 = PWMOutputDevice(16, active_high=True, initial_value=0)
        self.heater2 = PWMOutputDevice(19, active_high=True, initial_value=0)
        hats = hat_list()
        if not hats or hats[0].id != HatIDs.MCC_134:
            raise Exception("MCC134 not detected")
        self.mcc = mcc134(hats[0].address)
        self.mcc.tc_type_write(0, TcTypes.TYPE_K)
        self.mcc.tc_type_write(1, TcTypes.TYPE_K)

        # State
        self.tuner = None
        self.tuning_active = False
        self.selected_heater = 1
        self.tune_result = None
        self.pid1 = PIDController(2.0, 0.2, 0.1, output_min=0, output_max=1)
        self.pid2 = PIDController(2.0, 0.2, 0.1, output_min=0, output_max=1)
        self.test_active = False
        self.test_data = []

        self.init_ui()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_loop)
        self.timer.start(1000)

    def init_ui(self):
        layout = QVBoxLayout()
        tabs = QTabWidget()
        tabs.addTab(self.create_tuning_tab(), "Auto-Tune")
        tabs.addTab(self.create_test_tab(), "Test PID")
        tabs.addTab(self.create_manual_tab(), "Manual Tuning")
        layout.addWidget(tabs)
        self.setLayout(layout)

    def create_tuning_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()

        # Status display
        status_group = QGroupBox("Current Status")
        status_layout = QHBoxLayout()
        self.temp1_label = QLabel("Heater 1: --°C")
        self.temp2_label = QLabel("Heater 2: --°C")
        self.temp1_label.setStyleSheet("font-size: 18px; font-weight: bold;")
        self.temp2_label.setStyleSheet("font-size: 18px; font-weight: bold;")
        status_layout.addWidget(self.temp1_label)
        status_layout.addWidget(self.temp2_label)
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)

        # Config
        config_group = QGroupBox("Auto-Tune Configuration")
        config_layout = QHBoxLayout()
        config_layout.addWidget(QLabel("Heater:"))
        self.heater_combo = QComboBox()
        self.heater_combo.addItems(["Heater 1", "Heater 2"])
        config_layout.addWidget(self.heater_combo)
        config_layout.addWidget(QLabel("Setpoint (°C):"))
        self.setpoint_spin = QSpinBox()
        self.setpoint_spin.setRange(30, 100)
        self.setpoint_spin.setValue(50)
        config_layout.addWidget(self.setpoint_spin)
        config_layout.addWidget(QLabel("Hysteresis (°C):"))
        self.hysteresis_spin = QDoubleSpinBox()
        self.hysteresis_spin.setRange(0.5, 5.0)
        self.hysteresis_spin.setValue(1.0)
        self.hysteresis_spin.setSingleStep(0.5)
        config_layout.addWidget(self.hysteresis_spin)
        config_group.setLayout(config_layout)
        layout.addWidget(config_group)

        # Buttons
        btn_layout = QHBoxLayout()
        self.start_btn = QPushButton("Start Auto-Tune")
        self.start_btn.setStyleSheet("background-color: green; color: white; font-weight: bold; min-height: 40px;")
        self.start_btn.clicked.connect(self.start_tuning)
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.clicked.connect(self.stop_tuning)
        self.stop_btn.setEnabled(False)
        btn_layout.addWidget(self.start_btn)
        btn_layout.addWidget(self.stop_btn)
        layout.addLayout(btn_layout)

        # Progress
        self.progress_bar = QProgressBar()
        self.status_label = QLabel("Ready")
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.progress_bar)
        layout.addWidget(self.status_label)

        # Plot
        self.tune_figure = Figure(figsize=(8, 3))
        self.tune_canvas = FigureCanvas(self.tune_figure)
        self.tune_ax = self.tune_figure.add_subplot(111)
        layout.addWidget(self.tune_canvas)

        # Results
        results_group = QGroupBox("Tuning Results")
        results_layout = QVBoxLayout()
        self.results_text = QTextEdit()
        self.results_text.setReadOnly(True)
        self.results_text.setMaximumHeight(120)
        results_layout.addWidget(self.results_text)
        method_layout = QHBoxLayout()
        method_layout.addWidget(QLabel("Apply Method:"))
        self.method_combo = QComboBox()
        for m in TuningMethod:
            self.method_combo.addItem(m.value, m)
        self.method_combo.setCurrentIndex(1)
        method_layout.addWidget(self.method_combo)
        self.apply_btn = QPushButton("Apply to Selected Heater")
        self.apply_btn.clicked.connect(self.apply_tuning)
        self.apply_btn.setEnabled(False)
        method_layout.addWidget(self.apply_btn)
        self.save_btn = QPushButton("Save Results")
        self.save_btn.clicked.connect(self.save_results)
        self.save_btn.setEnabled(False)
        method_layout.addWidget(self.save_btn)
        results_layout.addLayout(method_layout)
        results_group.setLayout(results_layout)
        layout.addWidget(results_group)

        tab.setLayout(layout)
        return tab

    def create_test_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()

        # Current PID display
        pid_group = QGroupBox("Current PID Parameters")
        pid_layout = QHBoxLayout()
        self.pid1_label = QLabel("Heater 1: Kp=2.00, Ki=0.20, Kd=0.10")
        self.pid2_label = QLabel("Heater 2: Kp=2.00, Ki=0.20, Kd=0.10")
        pid_layout.addWidget(self.pid1_label)
        pid_layout.addWidget(self.pid2_label)
        pid_group.setLayout(pid_layout)
        layout.addWidget(pid_group)

        # Test config
        test_config = QGroupBox("Step Response Test")
        tc_layout = QHBoxLayout()
        tc_layout.addWidget(QLabel("Test Heater:"))
        self.test_heater_combo = QComboBox()
        self.test_heater_combo.addItems(["Heater 1", "Heater 2"])
        tc_layout.addWidget(self.test_heater_combo)
        tc_layout.addWidget(QLabel("Target (°C):"))
        self.test_target_spin = QSpinBox()
        self.test_target_spin.setRange(30, 100)
        self.test_target_spin.setValue(50)
        tc_layout.addWidget(self.test_target_spin)
        tc_layout.addWidget(QLabel("Duration (s):"))
        self.test_duration_spin = QSpinBox()
        self.test_duration_spin.setRange(60, 600)
        self.test_duration_spin.setValue(180)
        tc_layout.addWidget(self.test_duration_spin)
        test_config.setLayout(tc_layout)
        layout.addWidget(test_config)

        # Buttons
        test_btn_layout = QHBoxLayout()
        self.test_start_btn = QPushButton("Run Step Response Test")
        self.test_start_btn.setStyleSheet("background-color: #0066cc; color: white; font-weight: bold; min-height: 40px;")
        self.test_start_btn.clicked.connect(self.start_test)
        self.test_stop_btn = QPushButton("Stop Test")
        self.test_stop_btn.clicked.connect(self.stop_test)
        self.test_stop_btn.setEnabled(False)
        test_btn_layout.addWidget(self.test_start_btn)
        test_btn_layout.addWidget(self.test_stop_btn)
        layout.addLayout(test_btn_layout)

        # Plot
        self.test_figure = Figure(figsize=(8, 4))
        self.test_canvas = FigureCanvas(self.test_figure)
        self.test_ax = self.test_figure.add_subplot(111)
        layout.addWidget(self.test_canvas)

        # Metrics
        self.metrics_label = QLabel("Run a test to see response metrics")
        self.metrics_label.setStyleSheet("font-size: 14px; font-family: monospace;")
        layout.addWidget(self.metrics_label)

        tab.setLayout(layout)
        return tab

    def create_manual_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()

        for i, (pid, heater_name) in enumerate([(self.pid1, "Heater 1"), (self.pid2, "Heater 2")], 1):
            group = QGroupBox(f"{heater_name} PID Parameters")
            g_layout = QHBoxLayout()
            kp_spin = QDoubleSpinBox()
            kp_spin.setRange(0, 50)
            kp_spin.setValue(pid.kp)
            kp_spin.setSingleStep(0.1)
            ki_spin = QDoubleSpinBox()
            ki_spin.setRange(0, 10)
            ki_spin.setValue(pid.ki)
            ki_spin.setSingleStep(0.01)
            kd_spin = QDoubleSpinBox()
            kd_spin.setRange(0, 10)
            kd_spin.setValue(pid.kd)
            kd_spin.setSingleStep(0.01)
            g_layout.addWidget(QLabel("Kp:"))
            g_layout.addWidget(kp_spin)
            g_layout.addWidget(QLabel("Ki:"))
            g_layout.addWidget(ki_spin)
            g_layout.addWidget(QLabel("Kd:"))
            g_layout.addWidget(kd_spin)
            apply_btn = QPushButton("Apply")
            apply_btn.clicked.connect(lambda _, p=pid, kp=kp_spin, ki=ki_spin, kd=kd_spin: self.manual_apply(p, kp, ki, kd))
            g_layout.addWidget(apply_btn)
            group.setLayout(g_layout)
            layout.addWidget(group)
            if i == 1:
                self.manual_kp1, self.manual_ki1, self.manual_kd1 = kp_spin, ki_spin, kd_spin
            else:
                self.manual_kp2, self.manual_ki2, self.manual_kd2 = kp_spin, ki_spin, kd_spin

        layout.addStretch()
        tab.setLayout(layout)
        return tab

    def manual_apply(self, pid, kp_spin, ki_spin, kd_spin):
        pid.set_tunings(kp_spin.value(), ki_spin.value(), kd_spin.value())
        self.update_pid_labels()

    def update_pid_labels(self):
        self.pid1_label.setText(f"Heater 1: Kp={self.pid1.kp:.2f}, Ki={self.pid1.ki:.4f}, Kd={self.pid1.kd:.4f}")
        self.pid2_label.setText(f"Heater 2: Kp={self.pid2.kp:.2f}, Ki={self.pid2.ki:.4f}, Kd={self.pid2.kd:.4f}")

    def read_temp(self, channel):
        try:
            return self.mcc.t_in_read(channel)
        except Exception as e:
            print(f"Temp read error ch{channel}: {e}")
            return 25.0

    def update_loop(self):
        temp1, temp2 = self.read_temp(0), self.read_temp(1)
        self.temp1_label.setText(f"Heater 1: {temp1:.2f}°C")
        self.temp2_label.setText(f"Heater 2: {temp2:.2f}°C")

        if self.tuning_active and self.tuner:
            heater_idx = self.heater_combo.currentIndex()
            temp = temp1 if heater_idx == 0 else temp2
            heater = self.heater1 if heater_idx == 0 else self.heater2
            output, complete, result = self.tuner.update(temp)
            heater.value = output
            cycles = self.tuner._cycle_count
            self.progress_bar.setValue(min(100, int(cycles / self.tuner.min_cycles * 100)))
            self.status_label.setText(f"Tuning... Cycle {cycles}/{self.tuner.min_cycles} | Temp: {temp:.2f}°C | Output: {output*100:.0f}%")
            self.update_tune_plot()
            if complete:
                self.tuning_complete(result)

        if self.test_active:
            heater_idx = self.test_heater_combo.currentIndex()
            temp = temp1 if heater_idx == 0 else temp2
            pid = self.pid1 if heater_idx == 0 else self.pid2
            heater = self.heater1 if heater_idx == 0 else self.heater2
            output = pid.compute(temp)
            heater.value = output
            elapsed = time.time() - self.test_start_time
            self.test_data.append((elapsed, temp, pid.setpoint, output))
            self.update_test_plot()
            if elapsed >= self.test_duration_spin.value():
                self.test_complete()

    def start_tuning(self):
        self.tuner = RelayAutoTuner(
            setpoint=self.setpoint_spin.value(),
            output_min=0, output_max=1,
            hysteresis=self.hysteresis_spin.value(),
            min_cycles=4, max_cycles=10
        )
        self.tuning_active = True
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.apply_btn.setEnabled(False)
        self.save_btn.setEnabled(False)
        self.results_text.clear()
        self.progress_bar.setValue(0)

    def stop_tuning(self):
        self.tuning_active = False
        self.heater1.value = 0
        self.heater2.value = 0
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.status_label.setText("Tuning stopped")

    def tuning_complete(self, result):
        self.tuning_active = False
        self.heater1.value = 0
        self.heater2.value = 0
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.tune_result = result

        if result:
            self.apply_btn.setEnabled(True)
            self.save_btn.setEnabled(True)
            txt = f"Ultimate Gain (Ku): {result.ultimate_gain:.4f}\n"
            txt += f"Oscillation Period (Tu): {result.oscillation_period:.2f}s\n\n"
            for method, params in result.params.items():
                txt += f"{method.value}:\n  Kp={params.kp:.4f}, Ki={params.ki:.4f}, Kd={params.kd:.4f}\n"
            self.results_text.setText(txt)
            self.status_label.setText("Tuning complete! Select a method and apply.")
        else:
            self.status_label.setText("Tuning failed - try different settings")

    def apply_tuning(self):
        if not self.tune_result:
            return
        method = self.method_combo.currentData()
        params = self.tune_result.params[method]
        heater_idx = self.heater_combo.currentIndex()
        pid = self.pid1 if heater_idx == 0 else self.pid2
        pid.set_tunings(params.kp, params.ki, params.kd)
        self.update_pid_labels()
        # Update manual spinboxes
        if heater_idx == 0:
            self.manual_kp1.setValue(params.kp)
            self.manual_ki1.setValue(params.ki)
            self.manual_kd1.setValue(params.kd)
        else:
            self.manual_kp2.setValue(params.kp)
            self.manual_ki2.setValue(params.ki)
            self.manual_kd2.setValue(params.kd)
        self.status_label.setText(f"Applied {method.value} to Heater {heater_idx + 1}")

    def save_results(self):
        if not self.tune_result or not self.tuner:
            return
        fname = f"data/pid_autotune_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        try:
            with open(fname, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(["# PID Auto-Tune Results"])
                w.writerow([f"# Ku={self.tune_result.ultimate_gain:.4f}, Tu={self.tune_result.oscillation_period:.2f}"])
                for method, params in self.tune_result.params.items():
                    w.writerow([f"# {method.value}: Kp={params.kp:.4f}, Ki={params.ki:.4f}, Kd={params.kd:.4f}"])
                w.writerow(["Time_s", "Temperature_C", "Output", "Setpoint_C"])
                w.writerows(self.tuner.data_log)
            self.status_label.setText(f"Saved to {fname}")
        except Exception as e:
            self.status_label.setText(f"Save error: {e}")

    def update_tune_plot(self):
        if not self.tuner or not self.tuner.data_log:
            return
        self.tune_ax.clear()
        data = self.tuner.data_log
        t = [d[0] for d in data]
        temp = [d[1] for d in data]
        out = [d[2] * 100 for d in data]
        sp = [d[3] for d in data]
        self.tune_ax.plot(t, temp, 'b-', label='Temperature')
        self.tune_ax.plot(t, sp, 'r--', label='Setpoint')
        ax2 = self.tune_ax.twinx()
        ax2.fill_between(t, out, alpha=0.3, color='orange')
        ax2.set_ylabel('Output %')
        ax2.set_ylim(0, 120)
        self.test_ax.set_xlabel('Time (s)')
        self.test_ax.set_ylabel('Temperature (°C)')
        self.test_ax.legend(loc='upper left')
        self.test_canvas.draw()

    def calculate_metrics(self):
        if len(self.test_data) < 10:
            return
        temps = [d[1] for d in self.test_data]
        setpoint = self.test_data[0][2]
        initial = temps[0]

        # Overshoot
        max_temp = max(temps)
        overshoot = ((max_temp - setpoint) / (setpoint - initial) * 100) if max_temp > setpoint and setpoint != initial else 0

        # Rise time (10% to 90%)
        r10 = initial + 0.1 * (setpoint - initial)
        r90 = initial + 0.9 * (setpoint - initial)
        t10 = t90 = None
        for d in self.test_data:
            if t10 is None and d[1] >= r10:
                t10 = d[0]
            if t90 is None and d[1] >= r90:
                t90 = d[0]
        rise_time = (t90 - t10) if t10 and t90 else None

        # Settling time (2% band)
        tol = abs(setpoint - initial) * 0.02
        settle_time = None
        for i in range(len(temps) - 1, -1, -1):
            if abs(temps[i] - setpoint) > tol:
                if i + 1 < len(self.test_data):
                    settle_time = self.test_data[i + 1][0]
                break

        # Steady-state error
        ss_temps = temps[-20:] if len(temps) >= 20 else temps
        ss_error = setpoint - (sum(ss_temps) / len(ss_temps))

        txt = f"Overshoot: {overshoot:.1f}%\n"
        txt += f"Rise Time (10-90%): {rise_time:.1f}s\n" if rise_time else "Rise Time: N/A\n"
        txt += f"Settling Time (2%): {settle_time:.1f}s\n" if settle_time else "Settling Time: N/A\n"
        txt += f"Steady-State Error: {ss_error:.2f}°C"
        self.metrics_label.setText(txt)

    def closeEvent(self, event):
        self.heater1.value = 0
        self.heater2.value = 0
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = AutoTunerApp()
    window.show()
    sys.exit(app.exec_())