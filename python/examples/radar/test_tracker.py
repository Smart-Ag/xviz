import numpy as np
import matplotlib.pyplot as plt
from tracker import Tracker


def create_target_sequence(r, rdot, phi, theta):
    return [
        {
            'dr': r[i],
            'vr': rdot[i],
            'phi': phi[i],
            'elevation': theta[i],
        }
        for i in range(len(r))
    ]


def create_target_states(mu, sigma, n):
    r = np.random.normal(mu, sigma, n)
    rdot = np.random.normal(0, sigma, n)
    phi = np.random.normal(mu, sigma, n)
    theta = np.random.normal(mu, sigma, n)
    return r, rdot, phi, theta


def plot_target_measurment(target, target_key, r, c, ax):
    target_state = [t[target_key] for t in target]
    ax[r, c].plot(target_state)

def plot_track_state(track, track_idx, r, c, ax):
    track_state = [t[track_idx] for t in track] 
    ax[r, c].plot(track_state)

def main():
    n = 10
    mu, sigma = 10, .1
    r, rdot, phi, theta = create_target_states(mu, sigma, n)
    target_1 = create_target_sequence(r, rdot, phi, theta)

    mu, sigma = 12, .1
    r, rdot, phi, theta = create_target_states(mu, sigma, n)
    target_2 = create_target_sequence(r, rdot, phi, theta)

    target_set_sequence = zip(target_1, target_2)

    dt = 0.1
    cost_threshold = 16
    kill_threshold = 5
    tracker = Tracker(dt, cost_threshold, kill_threshold)

    track_1 = []
    track_2 = []
    for target_set in target_set_sequence:
        tracker.update(target_set)
        track_1.append(tracker.tracks[0].get_state().tolist())
        track_2.append(tracker.tracks[1].get_state().tolist())

    fig, ax = plt.subplots(nrows=2, ncols=3)

    plot_target_measurment(target_1, 'dr', 0, 0, ax)
    plot_target_measurment(target_2, 'dr', 0, 0, ax)
    plot_track_state(track_1, 0, 0, 0, ax)
    plot_track_state(track_2, 0, 0, 0, ax)

    plot_target_measurment(target_1, 'vr', 1, 0, ax)
    plot_target_measurment(target_2, 'vr', 1, 0, ax)
    plot_track_state(track_1, 1, 1, 0, ax)
    plot_track_state(track_2, 1, 1, 0, ax)

    plot_target_measurment(target_1, 'phi', 0, 1, ax)
    plot_target_measurment(target_2, 'phi', 0, 1, ax)
    plot_track_state(track_1, 2, 0, 1, ax)
    plot_track_state(track_2, 2, 0, 1, ax)

    plot_track_state(track_1, 3, 1, 1, ax)
    plot_track_state(track_2, 3, 1, 1, ax)

    plot_target_measurment(target_1, 'elevation', 0, 2, ax)
    plot_target_measurment(target_2, 'elevation', 0, 2, ax)
    plot_track_state(track_1, 4, 0, 2, ax)
    plot_track_state(track_2, 4, 0, 2, ax)

    plot_track_state(track_1, 5, 1, 2, ax)
    plot_track_state(track_2, 5, 1, 2, ax)

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
