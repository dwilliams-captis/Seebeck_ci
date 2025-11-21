#!/usr/bin/env python3
"""
PID Controller with Auto-Tuning for Raspberry Pi Heater Control
Uses relay-based auto-tuning (Åström-Hägglund method) to find optimal Kp, Ki, Kd
"""

import time
import math
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Optional

# Uncomment for actual Raspberry Pi usage:
import RPi.GPIO as GPIO


class TuningMethod(Enum):
    ZIEGLER_NICHOLS_CLASSIC = "zn_classic"
    ZIEGLER_NICHOLS_NO_OVERSHOOT = "zn_no_overshoot"
    TYREUS_LUYBEN = "tyreus_luyben"  # Good for minimal overshoot
    COHEN_COON = "cohen_coon"


@dataclass
class PIDParams:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0


@dataclass
class AutoTuneResult:
    ultimate_gain: float  # Ku
    oscillation_period: float  # Tu (seconds)
    params: dict = field(default_factory=dict)  # PID params for each method


class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, setpoint=0.0,
                 output_min=0.0, output_max=100.0, sample_time=0.1):
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
        
        # Anti-windup
        self._integral_max = output_max / max(ki, 0.001)
    
    def reset(self):
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = None
        self._last_input = None
    
    def set_tunings(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._integral_max = self.output_max / max(ki, 0.001)
    
    def compute(self, current_value: float, current_time: float = None) -> float:
        if current_time is None:
            current_time = time.time()
        
        error = self.setpoint - current_value
        
        if self._last_time is None:
            dt = self.sample_time
        else:
            dt = current_time - self._last_time
        
        if dt < self.sample_time:
            return self._clamp(self.kp * error)
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self._integral += error * dt
        self._integral = max(-self._integral_max, min(self._integral_max, self._integral))
        i_term = self.ki * self._integral
        
        # Derivative term (on measurement to avoid derivative kick)
        if self._last_input is not None:
            d_input = (current_value - self._last_input) / dt
            d_term = -self.kd * d_input
        else:
            d_term = 0.0
        
        output = p_term + i_term + d_term
        
        self._last_error = error
        self._last_time = current_time
        self._last_input = current_value
        
        return self._clamp(output)
    
    def _clamp(self, value: float) -> float:
        return max(self.output_min, min(self.output_max, value))


class RelayAutoTuner:
    """
    Relay-based auto-tuning using Åström-Hägglund method.
    Induces oscillations to determine ultimate gain and period.
    """
    
    def __init__(self, setpoint: float, output_min: float = 0.0,
                 output_max: float = 100.0, hysteresis: float = 0.5,
                 min_cycles: int = 4, max_cycles: int = 10,
                 timeout: float = 600.0):
        self.setpoint = setpoint
        self.output_min = output_min
        self.output_max = output_max
        self.hysteresis = hysteresis
        self.min_cycles = min_cycles
        self.max_cycles = max_cycles
        self.timeout = timeout
        
        self._relay_state = True  # True = high output
        self._peaks = []
        self._valleys = []
        self._zero_crossings = []
        self._last_value = None
        self._start_time = None
        self._cycle_count = 0
        self._increasing = None
    
    def reset(self):
        self._relay_state = True
        self._peaks = []
        self._valleys = []
        self._zero_crossings = []
        self._last_value = None
        self._start_time = None
        self._cycle_count = 0
        self._increasing = None
    
    def update(self, current_value: float, current_time: float = None) -> tuple:
        """
        Returns (output, is_complete, result)
        """
        if current_time is None:
            current_time = time.time()
        
        if self._start_time is None:
            self._start_time = current_time
        
        # Check timeout
        if current_time - self._start_time > self.timeout:
            return self.output_min, True, None
        
        error = self.setpoint - current_value
        
        # Relay with hysteresis
        if self._relay_state and error < -self.hysteresis:
            self._relay_state = False
        elif not self._relay_state and error > self.hysteresis:
            self._relay_state = True
        
        output = self.output_max if self._relay_state else self.output_min
        
        # Track peaks and valleys for oscillation analysis
        if self._last_value is not None:
            currently_increasing = current_value > self._last_value
            
            if self._increasing is not None:
                # Detect peak (was increasing, now decreasing)
                if self._increasing and not currently_increasing:
                    self._peaks.append((current_time, current_value))
                # Detect valley (was decreasing, now increasing)
                elif not self._increasing and currently_increasing:
                    self._valleys.append((current_time, current_value))
                    self._cycle_count += 1
            
            # Track zero crossings (crossing setpoint)
            if (self._last_value < self.setpoint <= current_value or
                self._last_value > self.setpoint >= current_value):
                self._zero_crossings.append(current_time)
            
            self._increasing = currently_increasing
        
        self._last_value = current_value
        
        # Check if we have enough data
        if self._cycle_count >= self.min_cycles and len(self._peaks) >= 3:
            result = self._calculate_result()
            if result is not None:
                return output, True, result
        
        if self._cycle_count >= self.max_cycles:
            result = self._calculate_result()
            return output, True, result
        
        return output, False, None
    
    def _calculate_result(self) -> Optional[AutoTuneResult]:
        if len(self._peaks) < 2 or len(self._valleys) < 2:
            return None
        
        # Calculate average amplitude
        peak_values = [p[1] for p in self._peaks[-4:]]
        valley_values = [v[1] for v in self._valleys[-4:]]
        amplitude = (sum(peak_values) / len(peak_values) - 
                    sum(valley_values) / len(valley_values)) / 2
        
        if amplitude < 0.01:
            return None
        
        # Calculate oscillation period from zero crossings or peaks
        if len(self._zero_crossings) >= 4:
            periods = []
            for i in range(2, len(self._zero_crossings)):
                periods.append(self._zero_crossings[i] - self._zero_crossings[i-2])
            tu = sum(periods) / len(periods)
        elif len(self._peaks) >= 2:
            periods = []
            for i in range(1, len(self._peaks)):
                periods.append(self._peaks[i][0] - self._peaks[i-1][0])
            tu = sum(periods) / len(periods)
        else:
            return None
        
        # Calculate ultimate gain using relay amplitude
        relay_amplitude = (self.output_max - self.output_min) / 2
        ku = (4 * relay_amplitude) / (math.pi * amplitude)
        
        # Calculate PID parameters using various methods
        result = AutoTuneResult(ultimate_gain=ku, oscillation_period=tu)
        result.params = self._calculate_pid_params(ku, tu)
        
        return result
    
    def _calculate_pid_params(self, ku: float, tu: float) -> dict:
        params = {}
        
        # Ziegler-Nichols Classic (may have ~25% overshoot)
        params[TuningMethod.ZIEGLER_NICHOLS_CLASSIC] = PIDParams(
            kp=0.6 * ku,
            ki=1.2 * ku / tu,
            kd=0.075 * ku * tu
        )
        
        # Ziegler-Nichols No Overshoot
        params[TuningMethod.ZIEGLER_NICHOLS_NO_OVERSHOOT] = PIDParams(
            kp=0.2 * ku,
            ki=0.4 * ku / tu,
            kd=0.066 * ku * tu
        )
        
        # Tyreus-Luyben (good for minimal overshoot)
        params[TuningMethod.TYREUS_LUYBEN] = PIDParams(
            kp=0.45 * ku,
            ki=0.54 * ku / tu,
            kd=0.0
        )
        
        # Cohen-Coon approximation
        params[TuningMethod.COHEN_COON] = PIDParams(
            kp=0.49 * ku,
            ki=0.98 * ku / tu,
            kd=0.06 * ku * tu
        )
        
        return params


class HeaterController:
    """Main controller class integrating PID and auto-tuning."""
    
    def __init__(self, pwm_pin: int = 18, pwm_freq: int = 100,
                 read_temp_func: Callable[[], float] = None,
                 sample_time: float = 0.5, simulation: bool = True):
        self.pwm_pin = pwm_pin
        self.pwm_freq = pwm_freq
        self.sample_time = sample_time
        self.simulation = simulation
        
        self.pid = PIDController(sample_time=sample_time)
        self.tuner = None
        self._tuning = False
        
        # For simulation/testing
        self._sim_temp = 25.0
        self._sim_ambient = 25.0
        self._sim_heater_power = 0.0
        
        if read_temp_func:
            self._read_temp = read_temp_func
        else:
            self._read_temp = self._simulate_temp_read
        
        if not simulation:
            self._setup_gpio()
    
    def _setup_gpio(self):
        """Initialize GPIO for PWM control."""
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        self._pwm = GPIO.PWM(self.pwm_pin, self.pwm_freq)
        self._pwm.start(0)
    
    def _simulate_temp_read(self) -> float:
        """Simple thermal simulation for testing."""
        # Thermal model: dT/dt = k1*power - k2*(T - ambient)
        k1 = 0.08  # Heating coefficient
        k2 = 0.02  # Cooling coefficient
        
        dt = self.sample_time
        heating = k1 * self._sim_heater_power * dt
        cooling = k2 * (self._sim_temp - self._sim_ambient) * dt
        
        self._sim_temp += heating - cooling
        # Add small noise
        import random
        return self._sim_temp + random.gauss(0, 0.1)
    
    def set_output(self, duty_cycle: float):
        """Set heater PWM duty cycle (0-100)."""
        duty_cycle = max(0, min(100, duty_cycle))
        self._sim_heater_power = duty_cycle
        
        if not self.simulation:
            self._pwm.ChangeDutyCycle(duty_cycle)
    
    def start_auto_tune(self, setpoint: float, hysteresis: float = 1.0):
        """Start the auto-tuning process."""
        self.tuner = RelayAutoTuner(
            setpoint=setpoint,
            output_min=0,
            output_max=100,
            hysteresis=hysteresis,
            min_cycles=4,
            max_cycles=10
        )
        self._tuning = True
        self.pid.setpoint = setpoint
        print(f"Starting auto-tune for setpoint {setpoint}°C...")
        print("This may take several minutes. Please wait.\n")
    
    def run_auto_tune(self) -> Optional[AutoTuneResult]:
        """Run the auto-tuning loop until complete."""
        if not self._tuning or self.tuner is None:
            return None
        
        data_log = []
        
        while self._tuning:
            current_temp = self._read_temp()
            current_time = time.time()
            
            output, complete, result = self.tuner.update(current_temp, current_time)
            self.set_output(output)
            
            data_log.append({
                'time': current_time,
                'temp': current_temp,
                'output': output
            })
            
            # Progress indicator
            if len(data_log) % 20 == 0:
                print(f"  Temp: {current_temp:.2f}°C | Output: {output:.1f}% | "
                      f"Cycles: {self.tuner._cycle_count}")
            
            if complete:
                self._tuning = False
                self.set_output(0)
                
                if result:
                    self._print_results(result)
                    return result
                else:
                    print("Auto-tuning failed. Try adjusting hysteresis or setpoint.")
                    return None
            
            time.sleep(self.sample_time)
        
        return None
    
    def _print_results(self, result: AutoTuneResult):
        print("\n" + "="*60)
        print("AUTO-TUNING COMPLETE")
        print("="*60)
        print(f"\nUltimate Gain (Ku): {result.ultimate_gain:.4f}")
        print(f"Oscillation Period (Tu): {result.oscillation_period:.2f} seconds\n")
        print("-"*60)
        print("Recommended PID Parameters:\n")
        
        for method, params in result.params.items():
            print(f"  {method.value}:")
            print(f"    Kp = {params.kp:.4f}")
            print(f"    Ki = {params.ki:.4f}")
            print(f"    Kd = {params.kd:.4f}\n")
        
        print("-"*60)
        print("RECOMMENDATION: For minimal overshoot, use 'zn_no_overshoot'")
        print("                For faster response, use 'tyreus_luyben'")
        print("="*60)
    
    def apply_tuning(self, result: AutoTuneResult, 
                     method: TuningMethod = TuningMethod.ZIEGLER_NICHOLS_NO_OVERSHOOT):
        """Apply auto-tuned parameters to the PID controller."""
        if method in result.params:
            params = result.params[method]
            self.pid.set_tunings(params.kp, params.ki, params.kd)
            print(f"\nApplied {method.value} tuning:")
            print(f"  Kp={params.kp:.4f}, Ki={params.ki:.4f}, Kd={params.kd:.4f}")
    
    def run_controlled(self, setpoint: float, duration: float = 300):
        """Run temperature control with current PID settings."""
        self.pid.setpoint = setpoint
        self.pid.reset()
        
        print(f"\nRunning PID control to {setpoint}°C for {duration}s...")
        print(f"Using Kp={self.pid.kp:.4f}, Ki={self.pid.ki:.4f}, Kd={self.pid.kd:.4f}\n")
        
        start_time = time.time()
        data_log = []
        
        while time.time() - start_time < duration:
            current_temp = self._read_temp()
            current_time = time.time()
            
            output = self.pid.compute(current_temp, current_time)
            self.set_output(output)
            
            elapsed = current_time - start_time
            error = setpoint - current_temp
            data_log.append({
                'time': elapsed,
                'temp': current_temp,
                'setpoint': setpoint,
                'output': output,
                'error': error
            })
            
            if len(data_log) % 10 == 0:
                print(f"  t={elapsed:.1f}s | Temp={current_temp:.2f}°C | "
                      f"Error={error:.2f}°C | Output={output:.1f}%")
            
            time.sleep(self.sample_time)
        
        self.set_output(0)
        self._analyze_response(data_log, setpoint)
        return data_log
    
    def _analyze_response(self, data: list, setpoint: float):
        """Analyze the step response metrics."""
        temps = [d['temp'] for d in data]
        times = [d['time'] for d in data]
        
        initial_temp = temps[0]
        max_temp = max(temps)
        final_temps = temps[-20:] if len(temps) >= 20 else temps
        steady_state = sum(final_temps) / len(final_temps)
        
        # Overshoot
        if max_temp > setpoint:
            overshoot = ((max_temp - setpoint) / (setpoint - initial_temp)) * 100
        else:
            overshoot = 0
        
        # Settling time (within 2% of setpoint)
        tolerance = abs(setpoint - initial_temp) * 0.02
        settling_time = None
        for i in range(len(temps) - 1, -1, -1):
            if abs(temps[i] - setpoint) > tolerance:
                if i + 1 < len(times):
                    settling_time = times[i + 1]
                break
        
        # Rise time (10% to 90%)
        range_10 = initial_temp + 0.1 * (setpoint - initial_temp)
        range_90 = initial_temp + 0.9 * (setpoint - initial_temp)
        t_10 = t_90 = None
        for i, temp in enumerate(temps):
            if t_10 is None and temp >= range_10:
                t_10 = times[i]
            if t_90 is None and temp >= range_90:
                t_90 = times[i]
        rise_time = (t_90 - t_10) if (t_10 and t_90) else None
        
        # Steady-state error
        ss_error = setpoint - steady_state
        
        print("\n" + "="*50)
        print("RESPONSE ANALYSIS")
        print("="*50)
        print(f"  Overshoot: {overshoot:.1f}%")
        print(f"  Rise Time (10-90%): {rise_time:.1f}s" if rise_time else "  Rise Time: N/A")
        print(f"  Settling Time (2%): {settling_time:.1f}s" if settling_time else "  Settling Time: N/A")
        print(f"  Steady-State Error: {ss_error:.2f}°C")
        print("="*50)
    
    def cleanup(self):
        """Clean up GPIO resources."""
        self.set_output(0)
        if not self.simulation:
            import RPi.GPIO as GPIO
            self._pwm.stop()
            GPIO.cleanup()


def main():
    """Example usage of the auto-tuning heater controller."""
    
    # Create controller (simulation mode for testing)
    controller = HeaterController(
        pwm_pin=18,
        sample_time=0.5,
        simulation=True  # Set False for real hardware
    )
    
    try:
        # Target temperature
        target_temp = 50.0
        
        # Step 1: Run auto-tuning
        controller.start_auto_tune(setpoint=target_temp, hysteresis=1.0)
        result = controller.run_auto_tune()
        
        if result:
            # Step 2: Apply the best tuning method
            controller.apply_tuning(result, TuningMethod.ZIEGLER_NICHOLS_NO_OVERSHOOT)
            
            # Step 3: Run controlled heating with tuned PID
            controller.run_controlled(setpoint=target_temp, duration=120)
    
    finally:
        controller.cleanup()


if __name__ == "__main__":
    main()

