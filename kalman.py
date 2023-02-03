import numpy as np

class KalmanFilter:
    def __init__(self, process_variance, estimated_measurement_variance):
        self.process_variance = process_variance
        self.estimated_measurement_variance = estimated_measurement_variance
        self.posteri_estimate = None
        self.posteri_error_estimate = None

    def input_latest_noisy_measurement(self, measurement):
        if self.posteri_estimate is None:
            self.posteri_estimate = measurement
            self.posteri_error_estimate = 1.0
        else:
            priori_estimate = self.posteri_estimate
            priori_error_estimate = self.posteri_error_estimate + self.process_variance

            blending_factor = priori_error_estimate / (priori_error_estimate + self.estimated_measurement_variance)
            self.posteri_estimate = priori_estimate + blending_factor * (measurement - priori_estimate)
            self.posteri_error_estimate = (1 - blending_factor) * priori_error_estimate

    def get_latest_estimated_measurement(self):
        return self.posteri_estimate
"""
This implementation assumes a constant velocity model for the process being filtered and a Gaussian noise model 
for the measurements. The process_variance and estimated_measurement_variance arguments to the constructor are used to tune the filter.

You can use this filter by creating an instance of the KalmanFilter class, and then repeatedly calling input_latest_noisy_measurement 
with the latest noisy measurement. The filtered value can be obtained at any time by calling get_latest_estimated_measurement.
"""