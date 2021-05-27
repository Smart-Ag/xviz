from scipy.optimize import linear_sum_assignment
from .tracks import SMTrack, BoschTrack, TrackType


class Tracker:

    def __init__(self, dt, updates_until_alive,
                 cost_threshold, kill_threshold, track_type):
        self.dt = dt  # timestep length
        self.updates_until_alive = updates_until_alive

        # if cost of a match is above this threshold, the target does not update
        # the track. Instead, a new track is spawned.
        self.cost_threshold = cost_threshold

        # kill track after it hasn't been updated after a certain number of
        # iterations
        self.kill_threshold = kill_threshold

        self.tracks = []
        self.track_type = track_type

    def update(self, targets, u=None):
        """
        1) Assign targets to tracks
        2) Update track from assigned target if assignment cost is below
        threshold. Otherwise, spawn new track for target and update track
        without measurement.
        3) Spawn new tracks for all unassigned targets
        4) Update unassigned tracks without using a measurement
        5) Kill old tracks
        """
        row_idx, col_idx, cost_matrix = self.assign(targets)
        target_idx_set = set(range(len(targets)))
        track_idx_set = set(range(len(self.tracks)))

        spawned_tracks = []
        for i in range(len(row_idx)):
            r, c = row_idx[i], col_idx[i]
            if cost_matrix[r][c] < self.cost_threshold:
                self.tracks[c].update(targets[r], u)
            else:  # bad match
                new_track = self.spawn_track(targets[r])
                spawned_tracks.append(new_track)
                self.tracks[c].blind_update(u)

            target_idx_set.remove(r)
            track_idx_set.remove(c)

        for target_idx in target_idx_set:
            unmatched_target = targets[target_idx]
            new_track = self.spawn_track(unmatched_target)
            spawned_tracks.append(new_track)

        for track_idx in track_idx_set:
            unmatched_track = self.tracks[track_idx]
            unmatched_track.blind_update(u)

        self.kill_old_tracks()

        self.tracks.extend(spawned_tracks)

    def kill_old_tracks(self):
        self.tracks = list(filter(
            lambda track: track.last_updated < self.kill_threshold,
            self.tracks))

    def spawn_track(self, target):
        """
        Inititialze the track with the target measurements
        """
        if self.track_type is TrackType.smart_micro:
            track = SMTrack(self.dt, self.updates_until_alive)
        elif self.track_type is TrackType.bosch:
            track = BoschTrack(self.dt, self.updates_until_alive)
        else:
            raise ValueError('track type is unrecognized')
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
                match_cost = track.compute_cost(target)
                row_cost.append(match_cost)
            cost_matrix.append(row_cost)
        row_idx, col_idx = linear_sum_assignment(cost_matrix)
        return row_idx, col_idx, cost_matrix

    def make_tracks_publishable(self):
        return list(map(lambda track: track.make_publishable(), self.tracks))
