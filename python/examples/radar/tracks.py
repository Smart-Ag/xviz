from enum import Enum
import numpy as np
from .kalman_filter import KalmanFilter
import uuid


class TrackType(Enum):
    smart_micro = 1
    bosch = 2


class Track:

    def __init__(self, dt, updates_until_alive):
        self.X, self.kf = self.initialize_kalman_filter(dt)
        self.is_alive = False
        self.updates_until_alive = updates_until_alive
        self.num_updates = 0
        self.last_updated = 0  # iterations since last updated by a target
        self.id = str(uuid.uuid4())

    def initialize_kalman_filter(self, dt):
        raise NotImplementedError()

    def get_state(self):
        return self.X

    def set_state(self, target):
        pass

    def get_measurement_array(self, target):
        raise NotImplementedError()

    def get_measurement_covariance(self, target):
        raise NotImplementedError()

    def compute_cost(self, target):
        """
        Cost is the sum of squared errors between the track state and the target
        measurement.
        Ex: C = (x1-x0)**2 + (y1-y0)**2 + (z1-z0)**2

        We might consider adding metadata differences to the cost
        """
        raise NotImplementedError()


    def update(self, target, u):
        if u is None:
            u = np.zeros(self.kf.G.shape[1])

        z = self.get_measurement_array(target)
        R = self.get_measurement_covariance(target)

        self.X = self.kf.estimate(z, u)

        self.num_updates += 1
        self.last_updated = 0

        if self.num_updates > self.updates_until_alive:
            self.is_alive = True

    def blind_update(self, u):
        """
        Update state based an pure system dynamics
        """
        if u is None:
            u = np.zeros(self.kf.G.shape[1])

        self.X = self.kf.blind_estimate(u)
        self.last_updated += 1

    def make_publishable(self):
        return {
            'state': self.get_state().tolist(),
            'is_alive': self.is_alive,
            'id': self.id,
        }


class SMTrack(Track):
    """
    States are spherical coordinates and the first time derivatives
    X = [r, rdot, phi, phidot, theta, thetadot]

    Measured values are a subset of the state vector
    z = [r, rdot, phi, theta]

    Assumptions:
    - zero acceleration throughout timestep
    - estimation errors for each axis are independent
    """

    def initialize_kalman_filter(self, dt):
        X = np.zeros(6)  # state vector

        F_tile = [
            [1, dt],
            [0, 1],
        ]
        # state transition matrix
        F = np.kron(np.eye(3), F_tile)

        # For now, disregard control input
        # Control input could be speed and yawrate of vehicle
        G = np.zeros((6, 2))

        # radar measures r, rdot, phi, and theta
        H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 1, 0],
        ])

        D = np.zeros((4, 2))  # no feedthrough

        state_estimation_variance = 1.
        # estimate uncertainty covariance
        P = np.diag(np.ones(6)) * state_estimation_variance

        acceleration_variance = .1
        Q_tile = [
            [dt**4/4, dt**3/2],
            [dt**3/2, dt**2],
        ]
        Q = np.kron(np.eye(3), Q_tile) * acceleration_variance

        measurement_variance = .1
        # measurement noise covariance
        R = np.diag(np.ones(4)) * measurement_variance

        return X, KalmanFilter(dt, X, F, G, H, D, P, Q, R)

    def set_state(self, target):
        self.X[0] = target['dr']
        self.X[1] = target['vr']
        self.X[2] = target['phi']
        self.X[4] = target['elevation']

    def get_measurement_array(self, target):
        return np.array([
            target['dr'],
            target['vr'],
            target['phi'],
            target['elevation'],
        ])

    def get_measurement_covariance(self, target):
        """
        SmartMicro does not provide standard deviation values so the measurement
        noise covariance matrix remains constant
        """
        return self.kf.R

    def compute_cost(self, target):
        track_r = self.X[0]
        target_r = target['dr']
        track_rdot = self.X[1]
        target_rdot = target['vr']
        track_phi = self.X[2]
        target_phi = target['phi']
        track_theta = self.X[4]
        target_theta = target['elevation']
        return (track_r - target_r)**2 \
               + (track_rdot - target_rdot)**2 \
               + (track_phi - target_phi)**2 \
               + (track_theta - target_theta)**2


class BoschTrack(Track):
    """
    States are polar coordinates and the first time derivatives
    X = [r, rdot, phi, phidot]

    Measured values are a subset of the state vector
    z = [r, rdot, phi]

    Assumptions:
    - zero acceleration throughout timestep
    - estimation errors for each axis are independent
    """

    def initialize_kalman_filter(self, dt):
        X = np.zeros(4)  # state vector

        F_tile = [
            [1, dt],
            [0, 1],
        ]
        # state transition matrix
        F = np.kron(np.eye(2), F_tile)

        # For now, disregard control input
        # Control input could be speed and yawrate of vehicle
        G = np.zeros((4, 2))

        # radar measures r, rdot, and phi
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
        ])

        D = np.zeros((2, 2))  # no feedthrough

        state_estimation_variance = 1.
        # estimate uncertainty covariance
        P = np.diag(np.ones(4)) * state_estimation_variance

        acceleration_variance = .1
        Q_tile = [
            [dt**4/4, dt**3/2],
            [dt**3/2, dt**2],
        ]
        Q = np.kron(np.eye(2), Q_tile) * acceleration_variance

        measurement_variance = .1
        # measurement noise covariance
        R = np.diag(np.ones(3)) * measurement_variance

        return X, KalmanFilter(dt, X, F, G, H, D, P, Q, R)

    def set_state(self, target):
        self.X[0] = target['dr']
        self.X[1] = target['vr']
        self.X[2] = target['phi']

    def get_measurement_array(self, target):
        return np.array([
            target['dr'],
            target['vr'],
            target['phi'],
        ])

    def get_measurement_covariance(self, target):
        """
        Bosch provides standard deviation values for r, rdot, and phi
        """
        r_sdv = target['drSdv']
        rdot_sdv = target['vrSdv']
        phi_sdv = target['phiSdv']
        return np.array([
            [r_sdv**2, rdot_sdv*r_sdv, 0],
            [r_sdv*rdot_sdv, rdot_sdv**2, 0],
            [0, 0, phi_sdv**2],
        ])

    def compute_cost(self, target):
        track_r = self.X[0]
        target_r = target['dr']
        track_rdot = self.X[1]
        target_rdot = target['vr']
        track_phi = self.X[2]
        target_phi = target['phi']
        return (track_r - target_r)**2 \
               + (track_rdot - target_rdot)**2 \
               + (track_phi - target_phi)**2
