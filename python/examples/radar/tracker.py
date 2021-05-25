from scipy.optimize import linear_sum_assignment
from .kalman_filter import KalmanFilter


class Tracker:
    """
    A track is a KalmanFilter class object. Use the state vector X from the
    KalmanFilter class to assign new targets to exisiting trackings where:
        X = [r, rdot, phi, phidot, theta, thetadot]
    """

    def __init__(self):
        self.tracks = []

    def assign_and_spawn(self, targets):
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
                row_cost.append(self.match_cost(target, track))
            cost_matrix.append(row_cost)

        row_idx, col_idx = linear_sum_assignment(cost_matrix)
        target_idx_set = set(len(targets))
        track_idx_set = set(len(self.tracks))

    def match_cost(target, track):
        """
        Cost is the sum of squares of position deltas:
            C = (x1-x0)**2 + (y1-y0)**2 + (z1-z0)**2

        We might consider adding metadata differences to the cost
        """
        track_r = track.X[0]
        target_r = target['dr']
        track_phi = track.X[2]
        target_phi = target['phi']
        track_theta = track.X[4]
        target_theta = target['elevation']
        return (track_r - target_r)**2 \
               + (track_phi - target_phi)**2 \
               + (track_theta - target_theta)**2
