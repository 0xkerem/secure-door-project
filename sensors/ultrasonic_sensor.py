# sensors/ultrasonic_sensor.py

import time
import pigpio
import numpy as np
from typing import Optional, Dict
from utils.filters import moving_average, SimpleKalmanFilter
from utils.logger import get_logger
from utils.gpio_pins import ULTRASONIC_TRIGGER, ULTRASONIC_ECHO

logger = get_logger(__name__)


class UltrasonicSensor:
    """
    Comprehensive HC-SR04 Ultrasonic Distance Sensor implementation with metrological specifications.
    
    Specifications Implemented:
    - Measurement Range: 2cm - 400cm
    - Output Range: 2cm - 400cm
    - Accuracy: ±1cm (theoretical), calibrated accuracy tracked
    - Calibration Error: Tracked during calibration
    - Hysteresis: Calculated during measurements
    - Saturation: Detected at range limits
    - Repeatability: Tracked over multiple measurements
    - Deadband: Configurable threshold for significant changes
    """

    # Sensor specifications
    MEASUREMENT_RANGE = (2.0, 400.0)  # cm
    THEORETICAL_ACCURACY = 1.0  # ±1cm
    SPEED_OF_SOUND = 34300  # cm/s at 20°C

    def __init__(
        self,
        trigger_pin: int = ULTRASONIC_TRIGGER,
        echo_pin: int = ULTRASONIC_ECHO,
        deadband_threshold: float = 0.5,
        use_kalman: bool = True,
        pi: Optional[pigpio.pi] = None
    ):
        """
        Initialize sensor with metrological tracking.
        
        Args:
            trigger_pin: GPIO pin for trigger
            echo_pin: GPIO pin for echo
            deadband_threshold: Minimum change to register as new value (cm)
            use_kalman: Whether to use Kalman filtering
            pi: Optional pigpio.pi instance (to allow sharing between multiple sensors)
        """
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.deadband_threshold = deadband_threshold
        self.speed_of_sound = self.SPEED_OF_SOUND
        # Timeout for max‐distance echo (400 cm ≃ 0.04 s)
        self.timeout_s = 0.04  # seconds

        # Metrological tracking
        self.last_value: Optional[float] = None
        self.calibration_error: Optional[float] = None
        self.hysteresis_history: list[float] = []
        self.repeatability_history: list[float] = []
        self.saturation_count: int = 0

        # Initialize filters
        self.kalman_filter = SimpleKalmanFilter() if use_kalman else None

        # pigpio initialization
        try:
            self.pi = pi if pi is not None else pigpio.pi()
            if not self.pi.connected:
                raise RuntimeError("pigpio daemon not running or can't connect.")
            self.pi.set_mode(self.trigger_pin, pigpio.OUTPUT)
            self.pi.set_mode(self.echo_pin, pigpio.INPUT)
            # Ensure trigger is low to start
            self.pi.write(self.trigger_pin, 0)
            time.sleep(0.5)  # Initial settling time
            logger.info("Ultrasonic sensor initialized with metrological tracking (pigpio)")
        except Exception as e:
            logger.error(f"Error initializing ultrasonic sensor: {str(e)}")
            raise

    def _get_raw_distance(self) -> Optional[float]:
        """
        Get a single raw distance measurement using pigpio's tick counters
        for microsecond‐precision timing.
        """
        try:
            # Generate a 10 µs trigger pulse automatically
            #   pigpio.gpio_trigger(pin, pulse_len, level)
            #   → raises(trigger HIGH for pulse_len µs, then automatically goes LOW)
            self.pi.gpio_trigger(self.trigger_pin, 10, 1)

            # Record the timestamp for timeout calculation (microseconds)
            start_tick = self.pi.get_current_tick()

            # Wait for ECHO to go HIGH (start of pulse)
            while self.pi.read(self.echo_pin) == 0:
                if pigpio.tickDiff(start_tick, self.pi.get_current_tick()) > int(self.timeout_s * 1e6):
                    # timed out waiting for echo to start
                    return None
            rise_tick = self.pi.get_current_tick()

            # Wait for ECHO to go LOW (end of pulse)
            while self.pi.read(self.echo_pin) == 1:
                if pigpio.tickDiff(rise_tick, self.pi.get_current_tick()) > int(self.timeout_s * 1e6):
                    # timed out waiting for echo to end
                    return None
            fall_tick = self.pi.get_current_tick()

            # Calculate pulse duration in microseconds
            pulse_width_us = pigpio.tickDiff(rise_tick, fall_tick)

            # Convert to seconds for distance formula
            pulse_duration_s = pulse_width_us / 1e6

            # Distance (cm) = (pulse_duration_s * speed_of_sound_cm_per_s) / 2
            distance = (pulse_duration_s * self.speed_of_sound) / 2.0

            # Check for saturation (below 2 cm or above 400 cm)
            if distance < self.MEASUREMENT_RANGE[0] or distance > self.MEASUREMENT_RANGE[1]:
                self.saturation_count += 1

            # Return distance if within valid range, else None
            if self.MEASUREMENT_RANGE[0] <= distance <= self.MEASUREMENT_RANGE[1]:
                return distance
            else:
                return None

        except Exception as e:
            logger.error(f"Measurement error: {str(e)}")
            return None

    def get_distance(self, num_samples: int = 5) -> Optional[float]:
        """
        Get filtered distance measurement with metrological tracking.
        
        Args:
            num_samples: Number of samples for averaging (if not using Kalman)
            
        Returns:
            Filtered distance in cm or None if measurement fails
        """
        try:
            measurements = []
            sample_count = 1 if self.kalman_filter else num_samples

            for _ in range(sample_count):
                dist = self._get_raw_distance()
                if dist is not None:
                    measurements.append(dist)
                time.sleep(0.25)  # small pause between raw reads
            
            if not measurements:
                return None

            # Apply filtering: either Kalman or simple moving average
            if self.kalman_filter:
                mean_val = np.mean(measurements)
                filtered_value = self.kalman_filter.update(mean_val)
            else:
                filtered_value = moving_average(measurements, len(measurements))

            # Deadband: if change is too small, keep last value
            if self.last_value is not None:
                if abs(filtered_value - self.last_value) < self.deadband_threshold:
                    return self.last_value

            # Track hysteresis (detect if direction changed)
            if self.last_value is not None:
                prev_value = self._get_prev_value(2)
                if prev_value is not None:
                    direction_change = (filtered_value - self.last_value) * (self.last_value - prev_value)
                    if direction_change < 0:  # sign flip → hysteresis event
                        self.hysteresis_history.append(abs(filtered_value - self.last_value))

            # Update repeatability history (keep only last 10)
            if len(self.repeatability_history) >= 10:
                self.repeatability_history.pop(0)
            self.repeatability_history.append(filtered_value)

            self.last_value = filtered_value
            return filtered_value

        except Exception as e:
            logger.error(f"Error in get_distance: {str(e)}")
            return None

    def calibrate(self, known_distance: float, num_samples: int = 20) -> Dict[str, float]:
        """
        Perform full sensor calibration and return metrological data.
        
        Args:
            known_distance: Precisely measured reference distance (cm)
            num_samples: Number of calibration samples
            
        Returns:
            Dictionary containing all calibration results:
            {
                'calibrated_speed': ...,
                'calibration_error': ...,
                'repeatability': ...,
                'hysteresis': ...,
                'accuracy': ...,
                'deadband': ...,
                'saturation_count': ...
            }
        """
        if not self.MEASUREMENT_RANGE[0] <= known_distance <= self.MEASUREMENT_RANGE[1]:
            raise ValueError("Known distance outside measurement range")

        self.hysteresis_history.clear()
        measurements = []

        for _ in range(num_samples):
            dist = self._get_raw_distance()
            if dist is not None:
                measurements.append(dist)
            time.sleep(0.1)

        if len(measurements) < num_samples / 2:
            raise RuntimeError("Insufficient valid measurements for calibration")

        avg_measured = np.mean(measurements)
        std_dev = np.std(measurements)

        # Adjust speed of sound based on known distance and measured average
        # new_speed = old_speed * (known / avg_measured)
        self.speed_of_sound = self.speed_of_sound * (known_distance / avg_measured)

        calibration_error = abs(avg_measured - known_distance)
        self.calibration_error = calibration_error
        repeatability = std_dev
        hysteresis = np.mean(self.hysteresis_history) if self.hysteresis_history else 0
        accuracy = max(self.THEORETICAL_ACCURACY, calibration_error)

        results = {
            'calibrated_speed': self.speed_of_sound,
            'calibration_error': calibration_error,
            'repeatability': repeatability,
            'hysteresis': hysteresis,
            'accuracy': accuracy,
            'deadband': self.deadband_threshold,
            'saturation_count': self.saturation_count
        }

        logger.info(f"Calibration complete. Results: {results}")
        return results

    def _get_prev_value(self, n: int = 1) -> Optional[float]:
        """Get nth previous value from repeatability history."""
        if len(self.repeatability_history) >= n:
            return self.repeatability_history[-n]
        return None

    def get_metrological_data(self) -> Dict[str, float]:
        """Get current metrological performance data."""
        repeatability = np.std(self.repeatability_history) if self.repeatability_history else 0
        hysteresis = np.mean(self.hysteresis_history) if self.hysteresis_history else 0

        return {
            'measurement_range': self.MEASUREMENT_RANGE,
            'output_range': self.MEASUREMENT_RANGE,
            'accuracy': max(self.THEORETICAL_ACCURACY, self.calibration_error or 0),
            'calibration_error': self.calibration_error or 0,
            'hysteresis': hysteresis,
            'saturation_count': self.saturation_count,
            'repeatability': repeatability,
            'deadband': self.deadband_threshold,
            'current_speed_of_sound': self.speed_of_sound,
        }

    def cleanup(self):
        """Clean up pigpio resources (disconnect if created here)."""
        try:
            if hasattr(self, "pi") and self.pi is not None:
                # Only stop the daemon if we started it here
                if not hasattr(self, "_external_pi") or not self._external_pi:
                    self.pi.stop()
            logger.info("pigpio cleanup done for ultrasonic sensor")
        except Exception as e:
            logger.error(f"pigpio cleanup error: {str(e)}")
