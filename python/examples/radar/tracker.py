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
        cost_matrix = []
        for track in self.tracks:
            row_cost = []
            for target in targets:
                row_cost.append(self.match_cost(track, target))

    def match_cost(track, target):
        """
        Cost is the sum of squares of position deltas:
            C = (x1-x0)**2 + (y1-y0)**2 + (z1-z0)**2

        We might consider adding metadata differences to the cost
        """
        pass
