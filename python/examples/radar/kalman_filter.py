import numpy as np
from scipy.linalg import inv


class KalmanFilter:

    def __init__(self, dt, X, F, G, H, D, P, Q, R):
        self.X = X  # state vector
        self.F = F  # state transition matrix
        self.G = G  # control matrix
        self.H = H  # observation matrix
        self.D = D  # feedthrough matrix
        self.P = P  # state covariance
        self.Q = Q  # process covariance
        self.R = R  # measurement covariance

    def update_state(self, K, z):
        """
        X + K * (z - H * X)
        """
        return self.X + K @ (z - self.H @ self.X)

    def update_estimate_covariance(self, K):
        """
        Unsimplified update equation for P:
        (I - K * H) * P * (I - K * H).T + K * R * K

        Simplified update equation for P
        (I - K * H) * P
        """
        # return (np.eye(6) - K @ self.H) @ self.P @ (np.eye(6) - K @ self.H).T \
        #         + K @ self.R @ K.T
        return (np.eye(6) - K @ self.H) @ self.P

    def predict_state(self, X, u):
        """
        F * X + G * u
        """
        return self.F @ X + self.G @ u

    def predict_estimate_covariance(self, P):
        """
        F * P * F.T + Q
        """
        return self.F @ P @ self.F.T + self.Q

    def calculate_kalman_gain(self):
        """
             P * H.T
        -----------------
        (H * P * H.T + R)
        """
        return self.P @ self.H.T @ inv(self.H @ self.P @ self.H.T + self.R)

    def update_with_measurement(self, z, u):
        """
        Update X(t) and P(t)
        Predict X(t+1) and P(t+1)
        Return X(t)
        """
        # update
        K = self.calculate_kalman_gain()
        X = self.update_state(K, z)
        P = self.update_estimate_covariance(K)

        # predict
        self.X = self.predict_state(X, u)
        self.P = self.predict_estimate_covariance(P)

        return X
