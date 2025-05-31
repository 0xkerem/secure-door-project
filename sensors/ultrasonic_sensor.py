# sensors/ultrasonic_sensor.py

import time
import RPi.GPIO as GPIO
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

    def __init__(self, 
                 trigger_pin: int = ULTRASONIC_TRIGGER,
                 echo_pin: int = ULTRASONIC_ECHO,
                 deadband_threshold: float = 0.5,
                 use_kalman: bool = True):
        """
        Initialize sensor with metrological tracking.
        
        Args:
            trigger_pin: GPIO pin for trigger
            echo_pin: GPIO pin for echo
            deadband_threshold: Minimum change to register as new value (cm)
            use_kalman: Whether to use Kalman filtering
        """
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.deadband_threshold = deadband_threshold
        self.speed_of_sound = self.SPEED_OF_SOUND
        self.timeout = 0.04  # 400cm max distance timeout in seconds
        
        # Metrological tracking
        self.last_value: Optional[float] = None
        self.calibration_error: Optional[float] = None
        self.hysteresis_history: list[float] = []
        self.repeatability_history: list[float] = []
        self.saturation_count: int = 0
        
        # Initialize filters
        self.kalman_filter = SimpleKalmanFilter() if use_kalman else None
        
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.trigger_pin, GPIO.OUT)
            GPIO.setup(self.echo_pin, GPIO.IN)
            GPIO.output(self.trigger_pin, False)
            time.sleep(0.5)  # Initial settling time
            logger.info("Ultrasonic sensor initialized with metrological tracking")
        except Exception as e:
            logger.error(f"Error initializing ultrasonic sensor: {str(e)}")
            raise

    def _get_raw_distance(self) -> Optional[float]:
        """Get single raw distance measurement with range checking."""
        try:
            # Send trigger pulse
            GPIO.output(self.trigger_pin, True)
            time.sleep(0.00001)
            GPIO.output(self.trigger_pin, False)

            # Wait for echo to go high
            timeout_start = time.time()
            while GPIO.input(self.echo_pin) == 0:
                if time.time() - timeout_start > self.timeout:
                    return None

            pulse_start = time.time()
            
            # Wait for echo to go low
            while GPIO.input(self.echo_pin) == 1:
                pulse_end = time.time()
                if pulse_end - pulse_start > self.timeout:
                    return None

            pulse_duration = pulse_end - pulse_start
            distance = (pulse_duration * self.speed_of_sound) / 2
            
            # Check saturation at range limits
            if distance <= self.MEASUREMENT_RANGE[0] or distance >= self.MEASUREMENT_RANGE[1]:
                self.saturation_count += 1
            
            # Return distance if valid, else None
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
                time.sleep(0.05)
            
            if not measurements:
                return None
            
            # Apply filtering
            if self.kalman_filter:
                # Kalman filter update with mean of current samples
                mean_val = np.mean(measurements)
                filtered_value = self.kalman_filter.update(mean_val)
            else:
                filtered_value = moving_average(measurements, len(measurements))
            
            # Deadband check: ignore minor fluctuations
            if self.last_value is not None:
                if abs(filtered_value - self.last_value) < self.deadband_threshold:
                    return self.last_value
            
            # Track hysteresis (detect direction changes)
            if self.last_value is not None:
                prev_value = self._get_prev_value(2)
                if prev_value is not None:
                    direction_change = (filtered_value - self.last_value) * (self.last_value - prev_value)
                    if direction_change < 0:  # Direction changed
                        self.hysteresis_history.append(abs(filtered_value - self.last_value))
            
            # Track repeatability - keep last 10 measurements
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
        # Formula: speed_of_sound_new = speed_of_sound_old * (known_distance / avg_measured)
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
        """Clean up GPIO settings."""
        GPIO.cleanup()
        logger.info("GPIO cleanup done for ultrasonic sensor")
