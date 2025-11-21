import sys
import csv
import time
from datetime import date
import random
import subprocess
from PyQt5.QtWidgets import (
    QApplication, QWidget, QTabWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QLineEdit, QPushButton, QTextEdit
)
from PyQt5.QtCore import QTimer, Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from gpiozero import PWMOutputDevice
from daqhats import mcc134, hat_list, HatIDs, TcTypes
from simple_pid import PID
import serial
import serial.tools.list_ports

# GPIO setup using gpiozero PWM
HEATER1_PIN = 16
HEATER2_PIN = 19
heater1 = PWMOutputDevice(HEATER1_PIN, active_high=True, initial_value=0)
heater2 = PWMOutputDevice(HEATER2_PIN, active_high=True, initial_value=0)

# MCC134 init
hats = hat_list()
if not hats or hats[0].id != HatIDs.MCC_134:
    raise Exception("MCC134 DaqHat not detected")
mcc = mcc134(hats[0].address)
mcc.tc_type_write(0, TcTypes.TYPE_K)
mcc.tc_type_write(1, TcTypes.TYPE_K)

# Multimeter init
BAUDS = [115200, 57600, 38400, 19200, 9600]

def find_candidate_ports():
    ports = []
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        if "gdm" in desc or "instek" in desc or "gw" in desc:
            ports.append(p.device)
    
    if not ports:
        for p in serial.tools.list_ports.comports():
            if p.device.startswith("/dev/ttyACM") or p.device.startswith("/dev/ttyUSB"):
                ports.append(p.device)
    
    return list(dict.fromkeys(ports))

def open_and_identify(port):
    for baud in BAUDS:
        try:
            ser = serial.Serial(port, baud, timeout=1, write_timeout=1)
        except Exception:
            continue
        
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            ser.write(b"*IDN?\r\n")
            time.sleep(0.15)
            resp = ser.readline().decode(errors="ignore").strip()
            
            if resp and ("GDM" in resp.upper() or "GW" in resp.upper()):
                print(f"Found GDM on {port} @ {baud} bps: {resp}")
                return ser
            
            ser.close()
        except Exception:
            try:
                ser.close()
            except Exception:
                pass
    
    return None

def find_multimeter():
    try:
        ports = find_candidate_ports()
        if not ports:
            print("No candidate serial ports found")
            return None
        
        for port in ports:
            ser = open_and_identify(port)
            if ser:
                try:
                    ser.write(b"SYST:REM\r\n")
                    time.sleep(0.1)
                    ser.write(b"CONF:AUTO 1\r\n")
                    time.sleep(0.1)
                    ser.write(b"TRIG:SOUR INT\r\n")
                    time.sleep(0.1)
                except Exception as e:
                    print(f"Warning: Error configuring multimeter: {e}")
                
                return ser
        
        print(f"No GDM instrument found on candidate ports: {ports}")
        return None
        
    except Exception as e:
        print(f"Error scanning for multimeter: {e}")
        return None

multimeter = find_multimeter()

class HeaterApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Heater Control UI")
        self.setFixedSize(800, 450)

        self.temp1 = 25.0
        self.temp2 = 25.0
        self.heating_active = False
        self.plotting_active = False
        self.temp_plotting_active = False
        self.show_temp1 = True
        self.show_temp2 = True
        self.voltage_data = []
        self.pid_logs = []
        self.voltage_history = []
        self.temp_data = []
        
        # Initialize setpoints before creating tabs
        self.setpoint1_value = self.round_to_nearest_5(25.0)
        self.setpoint2_value = self.round_to_nearest_5(25.0)
        
        # Seebeck cycle state variables
        self.seebeck_cycle_active = False
        self.cycle_stage = 0
        self.zero_crossing_count = 0
        self.last_error_sign = 0
        
        # Define new temperature sequence for Seebeck cycle
        # Each entry: (temp1_setpoint, temp2_setpoint, required_crossings)
        self.seebeck_temp_sequence = [
            (45, 30, 1),   # Stage 0: Heater 1 to 45C
            (40, 30, 1),   # Stage 1: Heater 1 to 40C
        ]

        self.tab_widget = QTabWidget()
        self.tab1 = self.create_tab1()
        self.tab2 = self.create_tab2()
        self.tab3 = self.create_tab3()
        self.tab4 = self.create_tab4()
        self.tab5 = self.create_tab5()
        self.tab6 = self.create_tab6()
        
        self.tab_widget.addTab(self.tab1, "Sample")
        self.tab_widget.addTab(self.tab2, "Heaters")
        self.tab_widget.addTab(self.tab3, "Seebeck Plot")
        self.tab_widget.addTab(self.tab4, "Voltage")
        self.tab_widget.addTab(self.tab5, "Temp Plot")
        self.tab_widget.addTab(self.tab6, "README")

        main_layout = QVBoxLayout()
        main_layout.addWidget(self.tab_widget)
        self.setLayout(main_layout)

        self.pid1 = PID(2.0, 0.2, 0.1, setpoint=25.0)
        self.pid2 = PID(2.0, 0.2, 0.1, setpoint=25.0)
        self.pid1.output_limits = (0, 1)
        self.pid2.output_limits = (0, 1)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_loop)
        self.timer.start(1000)

    def round_to_nearest_5(self, temperature):
        return round(temperature / 5) * 5

    def read_temperature(self, channel):
        try:
            return mcc.t_in_read(channel)
        except Exception as e:
            print(f"Error reading channel {channel}: {e}")
            return 0.0

    def send_cmd(self, ser, cmd, pause=0.08):
        ser.write((cmd + "\r\n").encode())
        time.sleep(pause)

    def read_line(self, ser):
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            line = ser.read_until(expected=b'\r').decode(errors="ignore").strip()
        return line

    def read_voltage(self):
        if multimeter is None:
            return 0.0
        
        try:
            self.send_cmd(multimeter, "*TRG")
            time.sleep(0.15)
            
            self.send_cmd(multimeter, "VAL1?")
            value_str = self.read_line(multimeter)
            
            if value_str:
                voltage_str = ''.join(c for c in value_str if c.isdigit() or c in '.-+eE')
                if voltage_str:
                    voltage_v = float(voltage_str)
                    return voltage_v * 1000.0
            
            return 0.0
                
        except Exception as e:
            print(f"Error reading voltage from multimeter: {e}")
            return 0.0

    def update_loop(self):
        self.temp1 = self.read_temperature(0)
        self.temp2 = self.read_temperature(1)
        
        voltage_mv = self.read_voltage()

        if hasattr(self, 'temp_display1'):
            self.temp_display1.setText(f"{self.temp1:.2f} °C")
        if hasattr(self, 'temp_display2'):
            self.temp_display2.setText(f"{self.temp2:.2f} °C")
        
        if hasattr(self, 'heater1_readout'):
            self.heater1_readout.setText(f"Heater 1: {self.temp1:.2f} °C")
        if hasattr(self, 'heater2_readout'):
            self.heater2_readout.setText(f"Heater 2: {self.temp2:.2f} °C")
        if hasattr(self, 'voltage_readout'):
            self.voltage_readout.setText(f"Voltage: {voltage_mv:.4f} mV")
        if hasattr(self, 'seebeck_readout'):
            seebeck_slope = self.calculate_seebeck_coefficient()
            self.seebeck_readout.setText(f"Seebeck: {seebeck_slope:.4f} mV/°C")
        
        if hasattr(self, 'voltage_display'):
            self.voltage_display.setText(f"{voltage_mv:.4f} mV")
        
        self.voltage_history.append(voltage_mv)
        if len(self.voltage_history) > 30:
            self.voltage_history.pop(0)
        
        self.update_voltage_stats()

        if self.heating_active:
            target1 = self.setpoint1_value
            target2 = self.setpoint2_value

            self.pid1.setpoint = target1
            self.pid2.setpoint = target2

            duty1 = self.pid1(self.temp1)
            duty2 = self.pid2(self.temp2)

            error1 = target1 - self.temp1
            error2 = target2 - self.temp2
            self.pid_logs.append((time.time(), self.temp1, target1, duty1, error1, self.temp2, target2, duty2, error2))

            heater1.value = duty1
            heater2.value = duty2
            
            if self.seebeck_cycle_active:
                self.manage_seebeck_cycle(error1)
        else:
            heater1.value = 0
            heater2.value = 0

        temp_diff = abs(self.temp1 - self.temp2)
        
        if self.plotting_active:
            timestamp = time.time()
            self.voltage_data.append((temp_diff, voltage_mv, self.temp1, self.temp2, timestamp))
        
        if self.temp_plotting_active:
            timestamp = time.time()
            self.temp_data.append((timestamp, self.temp1, self.temp2))
        
        self.update_plot(last_30_only=False)
        self.update_temp_plot()

    def manage_seebeck_cycle(self, error1):
        """Manage the automated Seebeck test cycle using temperature sequence array"""
        print(f"Current Error Sign: {current_error_sign}; Error1: {error1}")
        current_error_sign = 1 if error1 > 0 else -1
        
        # Detect zero crossing (error changes sign)
        if self.last_error_sign != 0 and current_error_sign != self.last_error_sign: #set point crossing occurred
            self.zero_crossing_count += 1
            
            # Get current stage requirements
            if self.cycle_stage < len(self.seebeck_temp_sequence):
                temp1_target, temp2_target, required_crossings = self.seebeck_temp_sequence[self.cycle_stage]
           
                print(f"Zero crossing detected! {error1} Count: {self.zero_crossing_count}/{required_crossings}, Stage: {self.cycle_stage}, Target: {temp1_target}°C")
                
                # Check if we've completed the required crossings for this stage
                if self.zero_crossing_count >= required_crossings:
                    # Move to next stage
                    self.cycle_stage += 1
                    self.zero_crossing_count = 0
                    
                    # Check if there's another stage
                    if self.cycle_stage < len(self.seebeck_temp_sequence):
                        # Set temperatures for next stage
                        next_temp1, next_temp2, next_crossings = self.seebeck_temp_sequence[self.cycle_stage]
                        
                        
                        self.setpoint1_value = next_temp1
                        self.setpoint2_value = next_temp2
                        self.update_setpoint_displays()

                        time.sleep(10)
                        print(f"Stage {self.cycle_stage - 1} complete. Moving to Stage {self.cycle_stage}: Set heater 1 to {next_temp1}°C, heater 2 to {next_temp2}°C [{error1}]")
                        print(f"New error: {self.setpoint1_value}")

                    else:
                        # All stages complete
                        print("All stages complete! Finishing Seebeck cycle...")
                        self.complete_seebeck_cycle()
        print(f"New setpoint value: {self.setpoint1_value}")                
        self.last_error_sign = current_error_sign

    def update_setpoint_displays(self):
        if hasattr(self, 'setpoint1_display'):
            self.setpoint1_display.setText(f"{self.setpoint1_value:.0f} °C")
        if hasattr(self, 'setpoint2_display'):
            self.setpoint2_display.setText(f"{self.setpoint2_value:.0f} °C")

    def complete_seebeck_cycle(self):
        self.seebeck_cycle_active = False
        self.cycle_stage = 0
        self.zero_crossing_count = 0
        self.last_error_sign = 0
        
        self.seebeck_status_label.setText("")
        self.run_seebeck_button.setEnabled(True)
        
        self.save_data()
        
        print("Seebeck cycle completed and data saved!")

    def start_seebeck_cycle(self):
        """Start the automated Seebeck test cycle"""
        if self.seebeck_cycle_active:
            return
        
        print("Starting Seebeck cycle...")
        print(f"Temperature sequence has {len(self.seebeck_temp_sequence)} stages")
        
        # Initialize cycle state
        self.seebeck_cycle_active = True
        self.cycle_stage = 0
        self.zero_crossing_count = 0
        self.last_error_sign = 0
        
        # Set initial setpoints from first stage in sequence
        initial_temp1, initial_temp2, initial_crossings = self.seebeck_temp_sequence[0]
        self.setpoint1_value = initial_temp1
        self.setpoint2_value = initial_temp2
        self.update_setpoint_displays()
        
        print(f"Stage 0: Setting heaters to {initial_temp1}°C and {initial_temp2}°C, waiting for {initial_crossings} crossings")
        
        # Start heating
        if not self.heating_active:
            self.start_heating()
        
        # Start temperature plotting
        if not self.temp_plotting_active:
            self.toggle_temp_plotting()
        
        # Start Seebeck plotting
        if not self.plotting_active:
            self.start_plotting()
        
        # Update UI
        self.seebeck_status_label.setText("Seebeck Cycle: ON")
        self.seebeck_status_label.setStyleSheet("font-size: 16px; font-weight: bold; color: green; background-color: #e0ffe0; padding: 5px;")
        self.run_seebeck_button.setEnabled(False)

    def increase_setpoint1(self):
        self.setpoint1_value += 5
        self.setpoint1_display.setText(f"{self.setpoint1_value:.0f} °C")

    def decrease_setpoint1(self):
        self.setpoint1_value = max(0, self.setpoint1_value - 5)
        self.setpoint1_display.setText(f"{self.setpoint1_value:.0f} °C")

    def increase_setpoint2(self):
        self.setpoint2_value += 5
        self.setpoint2_display.setText(f"{self.setpoint2_value:.0f} °C")

    def decrease_setpoint2(self):
        self.setpoint2_value = max(0, self.setpoint2_value - 5)
        self.setpoint2_display.setText(f"{self.setpoint2_value:.0f} °C")

    def create_tab1(self):
        tab = QWidget()
        layout = QVBoxLayout()

        layout.addWidget(QLabel("Sample Name:"))
        self.sample_name_field = QLineEdit()
        self.sample_name_field.setText(f"seebeck_{int(time.time())}_{random.randint(1000, 9999)}")
        layout.addWidget(self.sample_name_field)
        
        layout.addWidget(QLabel("Sample Description:"))
        self.sample_description_field = QTextEdit()
        self.sample_description_field.setMaximumHeight(80)
        layout.addWidget(self.sample_description_field)
        
        layout.addStretch()
        
        self.seebeck_status_label = QLabel("")
        self.seebeck_status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.seebeck_status_label)
        
        self.run_seebeck_button = QPushButton("Run Seebeck Test Cycle")
        self.run_seebeck_button.setStyleSheet("background-color: #0066cc; color: #000000; font-weight: bold; font-size: 18px; min-height: 60px;")
        self.run_seebeck_button.clicked.connect(self.start_seebeck_cycle)
        layout.addWidget(self.run_seebeck_button)
        
        self.manual_save_button = QPushButton("Save Current Data")
        self.manual_save_button.setStyleSheet("font-size: 12px; min-height: 30px;")
        self.manual_save_button.clicked.connect(self.save_data)
        layout.addWidget(self.manual_save_button)
        
        self.save_status_label = QLabel("")
        self.save_status_label.setStyleSheet("font-size: 12px; font-weight: bold;")
        self.save_status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.save_status_label)
        
        tab.setLayout(layout)
        return tab

    def create_tab2(self):
        tab = QWidget()
        layout = QVBoxLayout()

        row_layout = QHBoxLayout()
        col1 = QVBoxLayout()
        col2 = QVBoxLayout()

        col1.addWidget(QLabel("Setpoint Heater 1 (°C):"))
        
        setpoint1_controls = QHBoxLayout()
        self.setpoint1_down_btn = QPushButton("-5")
        self.setpoint1_down_btn.clicked.connect(self.decrease_setpoint1)
        self.setpoint1_display = QLabel(f"{self.setpoint1_value:.0f} °C")
        self.setpoint1_display.setStyleSheet("font-size: 16px; font-weight: bold; border: 1px solid black; padding: 5px;")
        self.setpoint1_display.setAlignment(Qt.AlignCenter)
        self.setpoint1_up_btn = QPushButton("+5")
        self.setpoint1_up_btn.clicked.connect(self.increase_setpoint1)
        
        setpoint1_controls.addWidget(self.setpoint1_down_btn)
        setpoint1_controls.addWidget(self.setpoint1_display)
        setpoint1_controls.addWidget(self.setpoint1_up_btn)
        col1.addLayout(setpoint1_controls)
        
        col1.addStretch(10)
        col1.addWidget(QLabel("Current Temp Heater 1:"))
        self.temp_display1 = QLabel("0.00 °C")
        self.temp_display1.setStyleSheet("font-size: 48px; font-weight: bold;")
        col1.addWidget(self.temp_display1)

        col2.addWidget(QLabel("Setpoint Heater 2 (°C):"))        
        setpoint2_controls = QHBoxLayout()
        self.setpoint2_down_btn = QPushButton("-5")
        self.setpoint2_down_btn.clicked.connect(self.decrease_setpoint2)
        self.setpoint2_display = QLabel(f"{self.setpoint2_value:.0f} °C")
        self.setpoint2_display.setStyleSheet("font-size: 16px; font-weight: bold; border: 1px solid black; padding: 5px;")
        self.setpoint2_display.setAlignment(Qt.AlignCenter)
        self.setpoint2_up_btn = QPushButton("+5")
        self.setpoint2_up_btn.clicked.connect(self.increase_setpoint2)
        
        setpoint2_controls.addWidget(self.setpoint2_down_btn)
        setpoint2_controls.addWidget(self.setpoint2_display)
        setpoint2_controls.addWidget(self.setpoint2_up_btn)
        col2.addLayout(setpoint2_controls)
        
        col2.addStretch(10)
        col2.addWidget(QLabel("Current Temp Heater 2:"))
        self.temp_display2 = QLabel("0.00 °C")
        self.temp_display2.setStyleSheet("font-size: 48px; font-weight: bold;")
        col2.addWidget(self.temp_display2)

        row_layout.addLayout(col1)
        row_layout.addLayout(col2)

        button_layout = QVBoxLayout()
        
        self.heating_status = QLabel("Heating: OFF")
        self.heating_status.setStyleSheet("font-size: 14px; color: red; font-weight: bold;")
        self.heating_status.setAlignment(Qt.AlignCenter)
        
        button_row_layout = QHBoxLayout()
        
        self.start_button = QPushButton("Start Heating")
        self.start_button.setStyleSheet("background-color: green; color: white; font-weight: bold; border: none; min-height: 60px;")
        self.start_button.clicked.connect(self.start_heating)

        self.stop_button = QPushButton("Stop Heating")
        self.stop_button.setStyleSheet("min-height: 60px;")
        self.stop_button.clicked.connect(self.stop_heating)

        button_row_layout.addWidget(self.start_button, 2)
        button_row_layout.addWidget(self.stop_button, 1)
        
        button_layout.addWidget(self.heating_status)
        button_layout.addLayout(button_row_layout)

        layout.addLayout(row_layout)
        layout.addLayout(button_layout)
        layout.addStretch(10)
        tab.setLayout(layout)
        return tab

    def start_heating(self):
        self.heating_active = True
        self.heating_status.setText("Heating: ON")
        self.heating_status.setStyleSheet("font-size: 14px; background-color: #00e500; color: white; font-weight: bold;")

    def stop_heating(self):
        self.heating_active = False
        heater1.value = 0
        heater2.value = 0
        self.heating_status.setText("Heating: OFF")
        self.heating_status.setStyleSheet("font-size: 14px; color: #ff0000; font-weight: bold;")

    def create_tab3(self):
        tab = QWidget()
        layout = QVBoxLayout()

        button_layout = QHBoxLayout()
        self.rescale_button = QPushButton("Re-Scale")
        self.rescale_button.clicked.connect(self.rescale_plot)
        
        self.show_all_button = QPushButton("Show All")
        self.show_all_button.clicked.connect(self.show_all_data)
        
        self.start_plotting_button = QPushButton("Start Plotting")
        self.start_plotting_button.setStyleSheet("font-weight: bold;")
        self.start_plotting_button.clicked.connect(self.start_plotting)
        
        button_layout.addWidget(self.rescale_button)
        button_layout.addWidget(self.show_all_button)
        button_layout.addWidget(self.start_plotting_button)
        button_layout.addStretch()
        
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)
        
        readout_layout = QHBoxLayout()
        
        self.heater1_readout = QLabel("Heater 1: -- °C")
        self.heater2_readout = QLabel("Heater 2: -- °C")
        self.voltage_readout = QLabel("Voltage: -- mV")
        self.seebeck_readout = QLabel("Seebeck: -- mV/°C")
        
        readout_layout.addWidget(self.heater1_readout)
        readout_layout.addWidget(self.heater2_readout)
        readout_layout.addWidget(self.voltage_readout)
        readout_layout.addWidget(self.seebeck_readout)

        layout.addLayout(button_layout)
        layout.addWidget(self.canvas)
        layout.addLayout(readout_layout)
        tab.setLayout(layout)
        return tab

    def calculate_seebeck_coefficient(self):
        if len(self.voltage_data) < 20:
            return 0.0
        
        recent_data = self.voltage_data[-30:]
        
        temp_diffs = [point[0] for point in recent_data]
        voltages = [point[1] for point in recent_data]
        
        if len(recent_data) < 2:
            return 0.0
        
        n = len(recent_data)
        sum_x = sum(temp_diffs)
        sum_y = sum(voltages)
        sum_xy = sum(temp_diffs[i] * voltages[i] for i in range(len(temp_diffs)))
        sum_x2 = sum(x * x for x in temp_diffs)
        
        denominator = n * sum_x2 - sum_x * sum_x
        if abs(denominator) < 1e-10:
            return 0.0
        
        slope = (n * sum_xy - sum_x * sum_y) / denominator
        return slope

    def rescale_plot(self):
        self.update_plot(last_30_only=True)

    def show_all_data(self):
        self.update_plot(last_30_only=False)

    def start_plotting(self):
        self.plotting_active = True
        self.voltage_data = []
        self.start_plotting_button.setText("Plotting Active")
        self.start_plotting_button.setStyleSheet("background-color: #0000ff; color: white; font-weight: bold;")
        self.ax.clear()
        self.canvas.draw()

    def update_plot(self, last_30_only=False):
        self.ax.clear()
        if self.voltage_data:
            if last_30_only and len(self.voltage_data) > 30:
                data_to_plot = self.voltage_data[-30:]
            else:
                data_to_plot = self.voltage_data
            
            x = [point[0] for point in data_to_plot]
            y = [point[1] for point in data_to_plot]
            
            self.ax.scatter(x, y, c='blue')
            self.ax.set_xlabel("Temperature Difference (°C)")
            self.ax.set_ylabel("Voltage (mV)")
            self.ax.set_title("Temperature Diff vs Voltage")
        self.canvas.draw()

    def create_tab4(self):
        tab = QWidget()
        layout = QHBoxLayout()

        left_layout = QVBoxLayout()
        voltage_label = QLabel("Current Voltage:")
        voltage_label.setStyleSheet("font-size: 24px; font-weight: bold;")
        
        self.voltage_display = QLabel("0.0 mV")
        self.voltage_display.setStyleSheet("font-size: 72px; font-weight: bold; color: black;")
        
        left_layout.addWidget(voltage_label)
        left_layout.addWidget(self.voltage_display)
        left_layout.addStretch()
        
        right_layout = QVBoxLayout()
        stats_title = QLabel("Statistics (Last 30 readings):")
        stats_title.setStyleSheet("font-size: 16px; font-weight: bold;")
        
        self.voltage_stats_table = QLabel()
        self.voltage_stats_table.setStyleSheet("font-size: 18px; font-family: monospace;")
        self.voltage_stats_table.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        
        right_layout.addWidget(stats_title)
        right_layout.addWidget(self.voltage_stats_table)
        right_layout.addStretch()
        
        layout.addLayout(left_layout, 2)
        layout.addLayout(right_layout, 1)
        
        tab.setLayout(layout)
        return tab

    def update_voltage_stats(self):
        if not hasattr(self, 'voltage_stats_table') or len(self.voltage_history) == 0:
            return
        
        avg_voltage = sum(self.voltage_history) / len(self.voltage_history)
        max_voltage = max(self.voltage_history)
        temp_diff = abs(self.temp1 - self.temp2)
        
        seebeck_coefficient = self.calculate_seebeck_coefficient()
        
        stats_text = f"""
Average Voltage:     {avg_voltage:.4f} mV
Maximum Voltage:     {max_voltage:.4f} mV
Temperature Diff:    {temp_diff:.2f} °C
Seebeck:             {seebeck_coefficient:.4f} mV/°C
Sample Count:        {len(self.voltage_history)}
        """.strip()
        
        self.voltage_stats_table.setText(stats_text)

    def create_tab5(self):
        tab = QWidget()
        layout = QVBoxLayout()

        button_layout = QHBoxLayout()
        
        self.temp_start_stop_button = QPushButton("Start Temperature Plotting")
        self.temp_start_stop_button.setStyleSheet("font-weight: bold;font-size:14px")
        self.temp_start_stop_button.clicked.connect(self.toggle_temp_plotting)
        
        self.toggle_temp1_button = QPushButton("Hide Temp 1")
        self.toggle_temp1_button.clicked.connect(self.toggle_temp1_display)
        
        self.toggle_temp2_button = QPushButton("Hide Temp 2")
        self.toggle_temp2_button.clicked.connect(self.toggle_temp2_display)
        
        button_layout.addWidget(self.temp_start_stop_button)
        button_layout.addWidget(self.toggle_temp1_button)
        button_layout.addWidget(self.toggle_temp2_button)
        button_layout.addStretch()
        
        self.temp_figure = Figure()
        self.temp_canvas = FigureCanvas(self.temp_figure)
        self.temp_ax = self.temp_figure.add_subplot(111)
        self.temp_ax.set_xlabel("Time (s)")
        self.temp_ax.set_ylabel("Temperature (°C)")
        self.temp_ax.set_title("Temperature vs Time")
        
        layout.addLayout(button_layout)
        layout.addWidget(self.temp_canvas)
        tab.setLayout(layout)
        return tab

    def toggle_temp_plotting(self):
        if self.temp_plotting_active:
            self.temp_plotting_active = False
            self.temp_data = []
            self.temp_start_stop_button.setText("Start Temperature Plotting")
            self.temp_start_stop_button.setStyleSheet("color: #0000ff; font-weight: bold;")
            
            self.temp_ax.clear()
            self.temp_ax.set_xlabel("Time (s)")
            self.temp_ax.set_ylabel("Temperature (°C)")
            self.temp_ax.set_title("Temperature vs Time")
            self.temp_canvas.draw()
        else:
            self.temp_plotting_active = True
            self.temp_data = []
            self.temp_start_stop_button.setText("Stop Temperature Plotting")
            self.temp_start_stop_button.setStyleSheet("background-color: #0000ff; color: white; font-weight: bold;")

    def toggle_temp1_display(self):
        self.show_temp1 = not self.show_temp1
        if self.show_temp1:
            self.toggle_temp1_button.setText("Hide Temp 1")
        else:
            self.toggle_temp1_button.setText("Show Temp 1")
        self.update_temp_plot()

    def toggle_temp2_display(self):
        self.show_temp2 = not self.show_temp2
        if self.show_temp2:
            self.toggle_temp2_button.setText("Hide Temp 2")
        else:
            self.toggle_temp2_button.setText("Show Temp 2")
        self.update_temp_plot()

    def update_temp_plot(self):
        self.temp_ax.clear()
        self.temp_ax.set_xlabel("Time (s)")
        self.temp_ax.set_ylabel("Temperature (°C)")
        self.temp_ax.set_title("Temperature vs Time")
        
        setpoint1 = self.setpoint1_value
        setpoint2 = self.setpoint2_value
        
        if self.temp_data:
            timestamps = [point[0] for point in self.temp_data]
            temp1_values = [point[1] for point in self.temp_data]
            temp2_values = [point[2] for point in self.temp_data]
            
            start_time = timestamps[0] if timestamps else 0
            time_relative = [(t - start_time) for t in timestamps]
            
            if self.show_temp1:
                self.temp_ax.plot(time_relative, temp1_values, 'r-', label='Heater 1', linewidth=2)
            if self.show_temp2:
                self.temp_ax.plot(time_relative, temp2_values, 'b-', label='Heater 2', linewidth=2)
            
            time_range = [min(time_relative), max(time_relative)]
            self.temp_ax.plot(time_range, [setpoint1, setpoint1], 'lightgrey', linestyle=':', 
                             label=f'Setpoint 1 ({setpoint1:.0f}°C)', linewidth=1.5)
            self.temp_ax.plot(time_range, [setpoint2, setpoint2], 'lightgrey', linestyle='--', 
                             label=f'Setpoint 2 ({setpoint2:.0f}°C)', linewidth=1.5)
            
            if self.show_temp1 or self.show_temp2:
                self.temp_ax.legend()
        else:
            self.temp_ax.axhline(y=setpoint1, color='lightgrey', linestyle=':', 
                               label=f'Setpoint 1 ({setpoint1:.0f}°C)', linewidth=1.5)
            self.temp_ax.axhline(y=setpoint2, color='lightgrey', linestyle='--', 
                               label=f'Setpoint 2 ({setpoint2:.0f}°C)', linewidth=1.5)
            self.temp_ax.legend()
        
        self.temp_canvas.draw()

    def create_tab6(self):
        tab = QWidget()
        layout = QVBoxLayout()
        self.readme_display = QTextEdit()
        self.readme_display.setReadOnly(True)
        try:
            with open("captis_sbr_readme.txt", "r") as f:
                self.readme_display.setText(f.read())
        except FileNotFoundError:
            self.readme_display.setText("README file not found.")
        layout.addWidget(self.readme_display)
        tab.setLayout(layout)
        return tab

    def save_data(self):
        try:
            today = date.today()
            description = self.sample_description_field.toPlainText().replace('\n', ' ')
            HeaderRow = f"Sample: {self.sample_name_field.text()}; Description: {description}; Date: {today.strftime('%m/%d/%y')}; Seebeck Calc:; Seebeck Units: mV/K"

            filename = "data/" + self.sample_name_field.text() + ".csv"
            with open(filename, "w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow([HeaderRow])
                writer.writerow(["Temperature_Difference_C", "Voltage_mV", "Temp1_C", "Temp2_C", "Timestamp"])
                writer.writerows(self.voltage_data)

            pid_logfile = "data/" + self.sample_name_field.text() + "_pidlog.csv"
            with open(pid_logfile, "w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(["Time", "Temp1", "Target1", "Duty1", "Error1", "Temp2", "Target2", "Duty2", "Error2"])
                writer.writerows(self.pid_logs)
                
            temp_logfile = "data/" + self.sample_name_field.text() + "_templog.csv"
            with open(temp_logfile, "w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(["Time", "Temp1", "Temp2"])
                writer.writerows(self.temp_data)
            
            self.save_status_label.setText("Files saved successfully!")
            self.save_status_label.setStyleSheet("font-size: 14px; font-weight: bold; color: green;")
            
            try:
                subprocess.run(["bash", "./gsynch.sh"], check=True)
                print("gsynch.sh executed successfully")
            except subprocess.CalledProcessError as e:
                print(f"Error executing gsynch.sh: {e}")
            except FileNotFoundError:
                print("gsynch.sh not found")
                
        except Exception as e:
            self.save_status_label.setText(f"Error saving files: {str(e)}")
            self.save_status_label.setStyleSheet("font-size: 14px; font-weight: bold; color: red;")

    def closeEvent(self, event):
        heater1.value = 0
        heater2.value = 0
        if multimeter is not None:
            try:
                self.send_cmd(multimeter, "SYST:LOC")
                multimeter.close()
            except:
                pass
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = HeaterApp()
    window.show()
    sys.exit(app.exec_())