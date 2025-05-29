# utils/filters.py
import numpy as np

def moving_average(data: list[float], window_size: int) -> float:
    """
    Calculates the moving average over the last N elements.

    :param data: List of numeric values (float or int).
    :param window_size: Number of elements to include in the average.
    :return: Averaged float value.
    """
    if not data or window_size <= 0:
        raise ValueError("Invalid data or window size.")
    return np.mean(data[-window_size:])


class SimpleKalmanFilter:
    """
    Simple 1D Kalman Filter implementation.

    Attributes:
        estimate (float): Current state estimate.
        error_estimate (float): Current estimation error.
        error_measurement (float): Measurement error (noise).
        kalman_gain (float): Weight of the measurement in the update.
    """

    def __init__(self, initial_estimate: float = 0.0,
                 error_estimate: float = 1.0,
                 error_measurement: float = 1.0):
        self.estimate = initial_estimate
        self.error_estimate = error_estimate
        self.error_measurement = error_measurement
        self.kalman_gain = 0.0

    def update(self, measurement: float) -> float:
        """
        Update the estimate with a new measurement.

        :param measurement: New sensor measurement.
        :return: Updated estimate.
        """
        self.kalman_gain = self.error_estimate / (
            self.error_estimate + self.error_measurement
        )
        self.estimate = self.estimate + self.kalman_gain * (measurement - self.estimate)
        self.error_estimate = (1 - self.kalman_gain) * self.error_estimate
        return self.estimate
