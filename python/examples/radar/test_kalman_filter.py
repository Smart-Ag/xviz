import numpy as np
import matplotlib.pyplot as plt
from kalman_filter import KalmanFilter


# testing
def main():
    dt = 0.1
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

    acceleration_variance = 1.
    # Q = np.diag(np.ones(6)) * acceleration_variance  # process noise covariance
    Q_tile = [
        [dt**4/4, dt**3/2],
        [dt**3/2, dt**2],
    ]
    Q = np.kron(np.eye(3), Q_tile) * acceleration_variance

    measurement_variance = .1
    # measurement noise covariance
    R = np.diag(np.ones(4)) * measurement_variance

    kf = KalmanFilter(dt, X, F, G, H, D, P, Q, R)

    u = np.zeros(2)

    mu, sigma = 10, .1
    x = np.random.normal(mu, sigma, 50)
    xdot = np.random.normal(0, sigma, 50)
    y = np.random.normal(mu, sigma, 50)
    z = np.random.normal(mu, sigma, 50)
    measurements = np.array([x, xdot, y, z]).T

    filtered_measurements = []

    for z in measurements:
        z_filtered = kf.update_with_measurement(z, u)
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
