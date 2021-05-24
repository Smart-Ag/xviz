import numpy as np
from scipy.signal import StateSpace
from scipy.linalg import inv


class Kalman:
    """
    States are spherical coordinates and the first time derivatives
    X = [r, rdot, phi, phidot, theta, thetadot]

    Measured values are a subset of the state vector
    z = [r, rdot, phi, theta]

    Assumptions:
    - zero acceleration throughout timestep
    - estimation errors for each axis are independent
    """

    def __init__(self, dt):
        A_tile = [
            [0, 1],
            [0, 0],
        ]
        A = np.kron(np.eye(3), A_tile)

        # For now, disregard control input
        # Control input could be speed and yawrate of vehicle
        B = np.zeros((6, 2))

        # radar measures r, rdot, phi, and theta
        C = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 1, 0],
        ])

        D = np.zeros((4, 2))  # no feedthrough

        # Discretize state space
        Gcont = StateSpace(A, B, C, D)
        Gdisc = Gcont.to_discrete(dt)
        self.F = Gdisc.A  # state transition matrix
        self.G = Gdisc.B  # control matrix
        self.H = Gdisc.C  # observation matrix
        self.D = Gdisc.D  # feedthrough matrix

        self.X = np.zeros(6)  # state vector

        state_estimation_variance = 1.
        # estimate uncertainty covariance
        self.P = np.diag(np.ones(6)) * state_estimation_variance

        acceleration_variance = 1.
        # self.Q = np.diag(np.ones(6)) * acceleration_variance  # process noise covariance
        Q_tile = [
            [dt**4/4, dt**3/2],
            [dt**3/2, dt**2],
        ]
        self.Q = np.kron(np.eye(3), Q_tile) * acceleration_variance

        measurement_variance = .1
        # measurement noise covariance
        self.R = np.diag(np.ones(4)) * measurement_variance

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

    def filter_measurement(self, z, u):
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


def main():
    kf = Kalman(.1)

    u = np.zeros(2)

    mu, sigma = 10, .1
    x = np.random.normal(mu, sigma, 50)
    xdot = np.random.normal(0, sigma, 50)
    y = np.random.normal(mu, sigma, 50)
    z = np.random.normal(mu, sigma, 50)
    measurements = np.array([x, xdot, y, z]).T

    filtered_measurements = []

    for z in measurements:
        z_filtered = kf.filter_measurement(z, u)
        filtered_measurements.append(z_filtered)

    fig, ax = plt.subplots(nrows=2, ncols=3)

    for j in range(3):
        for i in range(2):
            ax[i, j].plot([
                f[i+j*2]
                for f in
                filtered_measurements],
                label='kalman filtered')
            if not (j == 1 and i == 1) and not (j == 2 and i == 1):
                if j == 2:
                    idx = 3
                else:
                    idx = i+j*2
                ax[i, j].plot(measurements[:, idx], label='raw')
            ax[i, j].legend()

    plt.show()
    plt.close()


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    plt.rcParams['figure.figsize'] = [16, 10]
    plt.rcParams['savefig.facecolor'] = 'black'
    plt.rcParams['figure.facecolor'] = 'black'
    plt.rcParams['figure.edgecolor'] = 'white'
    plt.rcParams['axes.facecolor'] = 'black'
    plt.rcParams['axes.edgecolor'] = 'white'
    plt.rcParams['axes.labelcolor'] = 'white'
    plt.rcParams['axes.titlecolor'] = 'white'
    plt.rcParams['xtick.color'] = 'white'
    plt.rcParams['ytick.color'] = 'white'
    plt.rcParams['text.color'] = 'white'
    plt.rcParams["figure.autolayout"] = True
    # plt.rcParams['legend.facecolor'] = 'white'

    main()
