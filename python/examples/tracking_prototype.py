import time
from pathlib import Path
from scenarios.utils.filesystem import get_collector_instances, load_config, \
    load_global_config
from scenarios.utils.com_manager import ComManager, MqttConst
from scenarios.safety_subsystems.radar_filter import SmartMicroRadarFilter, \
    ThresholdFilter
from radar.read import get_target_sets
from radar.tracker import Tracker
from radar.tracks import TrackType


def publish_tracks(comm, tracks):
    comm.publish_json(MqttConst.RADAR_TRACKS_TOPIC, tracks)


def publish_none(comm):
    comm.publish(MqttConst.RADAR_TRACKS_TOPIC, None)


def main():
    configfile = Path(__file__).parent / 'scenarios' / 'collector-scenario-config.yaml'
    collector_config = load_config(str(configfile))
    global_config = load_global_config(collector_config['MACHINE_TYPE'])

    collector_output_file = collector_config['collector_output_file']
    extract_directory = collector_config['extract_directory']
    collector_instances = get_collector_instances(collector_output_file,
                                                  extract_directory)

    radar_filter = SmartMicroRadarFilter(dBpower_threshold=80.) \
        if collector_config['smartmicro_radar'] \
        else ThresholdFilter(global_config['safety']['radar'])

    raw_target_sets, filtered_target_sets = get_target_sets(collector_instances,
                                                            radar_filter)

    comm = ComManager()
    time.sleep(2.)  # allow time to connect to broker

    dt = 0.1
    cost_threshold = 2
    updates_until_alive = 7
    kill_threshold = 3
    track_type = TrackType.smart_micro \
        if collector_config['smartmicro_radar'] \
        else TrackType.bosch

    tracker = Tracker(dt, updates_until_alive,
                      cost_threshold, kill_threshold, track_type)

    for target_set in filtered_target_sets:
        if not target_set or target_set is None:
            publish_none(comm)
        else:
            tracker.update(target_set)
            publish_tracks(comm, tracker.make_tracks_publishable())

    time.sleep(0.5)


if __name__ == '__main__':
    main()
