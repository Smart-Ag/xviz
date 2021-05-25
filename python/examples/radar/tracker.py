import numpy as np
from scipy.optimize import linear_sum_assignment
from .kalman_filter import KalmanFilter


class Track:
    """
    States are spherical coordinates and the first time derivatives
    X = [r, rdot, phi, phidot, theta, thetadot]

    Measured values are a subset of the state vector
    z = [r, rdot, phi, theta]

    Assumptions:
    - zero acceleration throughout timestep
    - estimation errors for each axis are independent
    """

    def __init__(self, dt, updates_until_alive=5):
        self.X, self.kf = self.initialize_kalman_filter(dt)
        self.is_alive = False
        self.updates_until_alive = updates_until_alive
        self.num_updates = 0
        self.last_updated = 0  # iterations since last updated by a target

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

        return X, KalmanFilter(dt, X, F, G, H, D, P, Q, R)

    def get_state(self):
        return self.X

    def set_state(self, target):
        self.X[0] = target['dr']
        self.X[1] = target['vr']
        self.X[2] = target['phi']
        self.X[4] = target['elevation']

    def update(self, target, u):
        if u is None:
            u = np.zeros(self.kf.G.shape[1])
        z = np.array([
            target['dr'],
            target['vr'],
            target['phi'],
            target['elevation'],
        ])

        self.X = self.kf.update_with_measurement(z, u)

        self.num_updates += 1
        self.last_updated = 0

        if self.num_updates > self.updates_until_alive:
            self.is_alive = True


class Tracker:

    def __init__(self, dt, cost_threshold, kill_threshold):
        self.tracks = []  # list of Track objects
        self.dt = dt  # timestep length

        # if cost of a match is above this threshold, the target does not update
        # the track. Instead, a new track is spawned.
        self.cost_threshold = cost_threshold

        # kill track after it hasn't been updated after a certain number of
        # iterations
        self.kill_threshold = kill_threshold

    def update(self, targets, u=None):
        row_idx, col_idx, cost_matrix = self.assign(targets)
        target_idx_set = set(range(len(targets)))
        track_idx_set = set(range(len(self.tracks)))

        spawned_tracks = []
        for i in range(len(row_idx)):
            r, c = row_idx[i], col_idx[i]
            if cost_matrix[r, c] < self.cost_threshold:
                self.tracks[c].update(targets[r], u)
            else:  # bad match
                new_track = self.spawn_track(targets[r])
                spawned_tracks.append(new_track)
                self.tracks[c].last_updated += 1

            target_idx_set.remove(r)
            track_idx_set.remove(c)

        for target_idx in target_idx_set:
            unmatched_target = targets[target_idx]
            new_track = self.spawn_track(unmatched_target)
            spawned_tracks.append(new_track)

        for track_idx in track_idx_set:
            unmatched_track = self.tracks[track_idx]
            unmatched_track.last_updated += 1

        self.kill_old_tracks()

        self.tracks.extend(spawned_tracks)

    def kill_old_tracks(self):
        self.tracks = list(filter(
            lambda track: track.last_updated > self.kill_threshold,
            self.tracks))

    def spawn_track(self, target):
        """
        Inititialze the track with the target measurements
        """
        track = Track(self.dt)
        track.set_state(target)
        return track

    def assign(self, targets):
        """
        Cost matrix:
                 track1  track2 ..
                 -----------------
        target1 |      |       |
                 -----------------
        target2 |      |       |
                 -----------------
           :    |      |       |
        """
        cost_matrix = []
        for target in targets:
            row_cost = []
            for track in self.tracks:
                track_state = track.get_state()
                match_cost = self.compute_cost(target, track_state)
                row_cost.append(match_cost)
            cost_matrix.append(row_cost)
        row_idx, col_idx = linear_sum_assignment(cost_matrix)
        return row_idx, col_idx, np.array(cost_matrix)

    def compute_cost(self, target, track_state):
        """
        Cost is the sum of squares of position deltas:
            C = (x1-x0)**2 + (y1-y0)**2 + (z1-z0)**2

        We might consider adding metadata differences to the cost
        """
        track_r = track_state[0]
        target_r = target['dr']
        track_rdot = track_state[1]
        target_rdot = target['vr']
        track_phi = track_state[2]
        target_phi = target['phi']
        track_theta = track_state[4]
        target_theta = target['elevation']
        return (track_r - target_r)**2 \
               + (track_rdot - target_rdot)**2 \
               + (track_phi - target_phi)**2 \
               + (track_theta - target_theta)**2


# testing
def main():
    pass


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
