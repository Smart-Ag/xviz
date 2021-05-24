from pathlib import Path
from scenarios.utils.filesystem import get_collector_instances, load_config
from radar.read import get_target_sets


def main():
    configfile = Path(__file__).parent / 'scenarios' / 'collector-scenario-config.yaml'
    collector_config = load_config(str(configfile))

    collector_output_file = collector_config['collector_output_file']
    extract_directory = collector_config['extract_directory']
    collector_instances = get_collector_instances(collector_output_file,
                                                  extract_directory)

    targets, _ = get_target_sets(collector_instances)

    for target in targets:
        for t in target:
            print(t)
        break


if __name__ == '__main__':
    main()
