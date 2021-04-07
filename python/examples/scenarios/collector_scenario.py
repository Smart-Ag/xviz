import math
import time
from pathlib import Path
from collections import deque
import numpy as np
import cv2
import csv
import os

from google.protobuf.json_format import MessageToDict
from protobuf_APIs import falconeye_pb2, radar_pb2

from scenarios.meta.collector_meta import get_builder
from scenarios.utils.com_manager import ComManager, MqttConst
from scenarios.utils.filesystem import get_collector_instances, load_config, load_global_config
from scenarios.utils.gis import transform_combine_to_local, get_combine_region, \
    get_auger_region, utm_array_to_local, lonlat_array_to_local, \
    lonlat_to_utm, get_wheel_angle
from scenarios.utils.image import postprocess, show_image
from scenarios.utils.read_protobufs import deserialize_collector_output, \
    extract_collector_output, extract_collector_output_slim

from scenarios.safety_subsystems.radar_filter import RadarFilter
from scenarios.safety_subsystems.path_prediction import get_path_distances, get_path_poly, predict_path

from scenarios.safety_subsystems.real_path_prediction import get_rows, is_close_to_curve, get_actual_path_poly, closest_node

import xviz_avs as xviz

class CollectorScenario:

    def __init__(self, live=True, duration=10):
        self._timestamp = time.time()
        self._duration = duration
        self._live = live
        self._metadata = None
        self.index = 0
        self.data = []
        self.track_history = {}
        self.START_WRITING = True

        configfile = Path(__file__).parent / 'collector-scenario-config.yaml'
        collector_config = load_config(str(configfile))

        collector_output_file = collector_config['collector_output_file']
        extract_directory = collector_config['extract_directory']
        self.collector_instances = get_collector_instances(
            collector_output_file, extract_directory)

        self.mqtt_enabled = collector_config['mqtt_enabled']
        if self.mqtt_enabled:
            self.mqtt_tracking_outputs = []
            comm = ComManager()
            comm.subscribe(MqttConst.TRACKS_TOPIC, self.store_tracking_output)


        self.global_config = load_global_config(collector_config['MACHINE_TYPE'])
        print("Global config:", self.global_config)
        self.radar_filter = RadarFilter(self.global_config['safety']['radar'])
        self.cab_to_nose = self.global_config['safety']['object_tracking']['cabin_to_nose_distance']
        self.combine_dimensions = self.global_config['safety']['combine_dimensions']
        self.tractor_gps_to_rear_axle = self.global_config['safety']['tractor_dimensions']['gps_to_rear_axle']
        self.header_width = 8.0 # default, gets updated by machine state message

        self.tractor_state = deque(maxlen=10)
        self.tractor_easting = None
        self.tractor_northing = None
        self.tractor_theta = None
        self.combine_states = {}
        self.combine_x = None
        self.combine_y = None
        self.combine_relative_theta = None
        self.utm_zone = ''
        self.planned_path = None
        self.field_definition = None
        self.sync_status = None
        self.control_signal = None
        self.sync_params = None


    def reset_values(self):
        self.tractor_state.clear()
        self.tractor_easting = None
        self.tractor_northing = None
        self.tractor_theta = None
        self.combine_states = {}
        self.combine_x = None
        self.combine_y = None
        self.combine_relative_theta = None
        self.planned_path = None
        self.field_definition = None
        self.sync_status = None
        self.control_signal = None
        self.sync_params = None
        self.index = 0


    def count_uniques(self, arr2D):
        return len(np.unique(arr2D, axis=0))

    def store_tracking_output(self, msg):
        tracking_output = falconeye_pb2.TrackingOutput()
        tracking_output.ParseFromString(msg.payload)
        tracking_output = MessageToDict(tracking_output, including_default_value_fields=True)
        self.mqtt_tracking_outputs.append(tracking_output)
        print('tracking outputs received:', len(self.mqtt_tracking_outputs))


    def get_metadata(self):
        if not self._metadata:
            builder = get_builder()
            self._metadata = builder.get_message()

        metadata = {
            'type': 'xviz/metadata',
            'data': self._metadata.to_object()
        }

        if not self._live:
            log_start_time = self._timestamp
            metadata['data']['log_info'] = {
                "log_start_time": log_start_time,
                "log_end_time": log_start_time + self._duration
            }

        return metadata


    def get_message(self, time_offset):
        try:
            timestamp = self._timestamp + time_offset

            builder = xviz.XVIZBuilder(metadata=self._metadata)
            self._draw_measuring_references(builder, timestamp)
            self._draw_collector_instance(builder, timestamp)
            data = builder.get_message()

            return {
                'type': 'xviz/state_update',
                'data': data.to_object()
            }
        except Exception as e:
            print("Crashed in get_message:", e)
            raise e


    def _draw_measuring_references(self, builder: xviz.XVIZBuilder, timestamp):
        radial_distances = set([5, 10, 15, 20, 25, 30])
        radial_distances = sorted(radial_distances, reverse=True)

        for r in radial_distances:
            builder.primitive('/measuring_circles_lbl')\
                .text(str(r))\
                .position([r, 0, .1])\
                .id(f'{r}lb')

            builder.primitive('/measuring_circles')\
                .circle([0, 0, 0], r)\
                .id(str(r))

        cam_fov = [-28.5, 28.5]  # 57 deg
        radar_fov = [-42.6, -13.5, -6.75, 0, 6.75, 13.5, 42.6]  # 54 degrees

        for c_phi in cam_fov:
            r = 40
            label = (r, c_phi)
            (x, y, z) = self.get_object_xyz_primitive(r, c_phi*math.pi/180)
            vertices = [0, 0, 0, x, y, z]
            builder.primitive('/camera_fov')\
                .polyline(vertices)\
                .id("cam_fov: "+str(label))

        for r_phi in radar_fov:
            r = 40
            (x, y, z) = self.get_object_xyz_primitive(r, r_phi*math.pi/180)
            x += self.cab_to_nose
            builder.primitive('/measuring_circles_lbl')\
                .text(str(r_phi))\
                .position([x, y, z])\
                .id(str(r_phi)+'lb')
            if r_phi == radar_fov[0] or r_phi == radar_fov[-1]:
                label = (r, r_phi)
                vertices = [0, 0, 0, x, y, z]
                builder.primitive('/radar_fov')\
                    .polyline(vertices)\
                    .id("radar_fov: "+str(label))


    def _draw_collector_instance(self, builder: xviz.XVIZBuilder, timestamp):
        try:
            if self.index == len(self.collector_instances) - 100:
                self.reset_values()
                print("#############################WE FINISHED#################################")

            collector_output = self.collector_instances[self.index]

            collector_output, is_slim_output = deserialize_collector_output(collector_output)
            if is_slim_output:
                imgs, camera_output, radar_output, tracking_output, machine_state,\
                    field_definition, planned_path, sync_status, control_signal, sync_params \
                    = extract_collector_output_slim(collector_output)
            else:
                img, camera_output, radar_output,\
                    tracking_output, machine_state = extract_collector_output(collector_output)
                field_definition = None
                planned_path = None
                sync_status = None
                control_signal = None
                sync_params = None

            if machine_state is not None:
                self.update_machine_state(machine_state)

            if self.tractor_state:
                _, tractor_state = self.tractor_state[-1]
                self.tractor_theta = (90 - tractor_state['heading']) * math.pi / 180
                self.tractor_easting, self.tractor_northing = lonlat_to_utm(
                    tractor_state['longitude'],
                    tractor_state['latitude'],
                    self.utm_zone
                )
                tractor_speed = tractor_state['speed']

                builder.pose("/vehicle_pose")\
                    .timestamp(timestamp)\
                    .position(0., 0., 0.)\
                    .orientation(
                        tractor_state['roll'],
                        tractor_state['pitch'],
                        self.tractor_theta
                    )
                builder.primitive('/tractor_speed')\
                .text("T speed: " + str(round(tractor_speed, 3)))\
                .position([-self.tractor_gps_to_rear_axle-10, 10., 1.])\
                .id('tractor speed')

                wheel_angle = get_wheel_angle(
                    #tractor_state['curvature'] * 1000 * 1000,
                    tractor_state['curvature'],
                    self.global_config['guidance']['wheel_base'],
                )

                # Curvature = YawRate/Velocity


                builder.primitive('/tractor_curv')\
                .text("T curv: " + str(round(wheel_angle, 3)))\
                .position([-self.tractor_gps_to_rear_axle+10, 10., 1.])\
                .id('tractor curv')

            else:
                builder.pose("/vehicle_pose")\
                    .timestamp(timestamp)\

            if self.mqtt_enabled:
                if self.mqtt_tracking_outputs:
                    tracking_output = self.mqtt_tracking_outputs[self.index]
                else:
                    print("mqtt enabled but no mqtt tracking outputs are stored")
                    tracking_output = None

            if planned_path is not None:
                if planned_path.size > 0:
                    self.planned_path = planned_path.reshape(-1, 2)
                else:
                    self.planned_path = None

            if field_definition is not None:
                self.field_definition = field_definition

            if control_signal is not None:
                self.control_signal = control_signal

            if sync_status is not None:
                self.sync_status = sync_status

            if sync_params is not None:
                if sync_params:
                    self.sync_params = sync_params
                else:
                    self.sync_params = None

            self._draw_combine(builder)
            self._draw_auger(builder)
            self._draw_tracking_targets(tracking_output, builder)
            self._draw_camera_targets(camera_output, builder)
            self._draw_radar_targets(radar_output, builder)
            self._draw_predicted_paths(builder)
            self._draw_planned_path(builder)
            self._draw_field_definition(builder)
            self._draw_control_signal(builder)
            self._draw_sync_status(builder)
            self._draw_sync_params(builder)

            if sorted(imgs):
                conglomerate = imgs[0][1]  # primary cam image
                if camera_output is not None:
                    conglomerate = postprocess(conglomerate, camera_output)
                # if len(imgs) > 1:
                #     for _cam_index, img in imgs[1:]:
                #         conglomerate = np.vstack((conglomerate, img))
                show_image(conglomerate)


                #for _cam_index, img in imgs:
                #    tdir = str(f"cam-{_cam_index}")
                    #if not os.path.exists(tdir):
                    #    os.makedirs(tdir)
                    #out_path = os.path.join(tdir, f"{self.index}.jpeg")
                    #t_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    #cv2.imwrite(out_path, t_img)

            # if self.index == 0:
            #     print('start time:', time.gmtime(float(collector_output.timestamp)))
            # elif self.index == len(self.collector_instances) - 1:
            #     print('end time:', time.gmtime(float(collector_output.timestamp)))

            self.index += 1

        except Exception as e:
            print('Crashed in draw collector instance:', e)
            raise e


    def _draw_radar_targets(self, radar_output, builder: xviz.XVIZBuilder):

        if radar_output is None:
            return
        if self.sync_status is None:
            sync_status = dict(runningSync=False, inSync=False, atSyncPoint=False)
        else:
            sync_status = self.sync_status

        try:
            if self.radar_filter.prev_target_set is not None:
                if self.radar_filter.prev_target_set == radar_output['targets']:
                    return
            self.radar_filter.prev_target_set = radar_output['targets']

            for target in radar_output['targets'].values():
                (x, y, z) = self.get_object_xyz(target, 'phi', 'dr', radar_ob=True)

                _phi = target['phi']
                _dr = target['dr']

                if self.radar_filter.is_valid_target(target, sync_status=sync_status):
                    builder.primitive('/radar_passed_filter_targets')\
                        .circle([x, y, z+.1], .5)\
                        .id(str(target['targetId']))
                else:
                    if not target['consecutive'] < 1:
                        builder.primitive('/radar_filtered_out_targets')\
                            .circle([x, y, z], .5)\
                            .id(str(target['targetId']))
                    else:
                        pass

                if not target['consecutive'] < 1:
                    builder.primitive('/radar_id')\
                        .text(str(target['targetId']))\
                        .position([x, y, z+.2])\
                        .id(str(target['targetId']))

            for not_received_id in self.radar_filter.target_id_set:
                default_target = MessageToDict(
                    radar_pb2.RadarOutput.Target(),
                    including_default_value_fields=True
                )
                self.radar_filter.update_queue(not_received_id, default_target, sync_status)
            # reset the target id set for next cycle
            self.radar_filter.target_id_set = set(range(48))

        except Exception as e:
            print('Crashed in draw radar targets:', e)


    def _draw_tracking_targets(self, tracking_output, builder: xviz.XVIZBuilder):
        if tracking_output is None:
            return
        try:
            if 'tracks' in tracking_output:
                min_confidence = 0.1
                min_hits = 2
                for track in tracking_output['tracks']:
                    if not track['score'] > min_confidence \
                            or not track['hitStreak'] > min_hits:
                        continue

                    (x, y, z) = self.get_object_xyz(track, 'angle', 'distance', radar_ob=False)

                    if track['radarDistCamFrame'] != self.track_history.get(track['trackId'], -1) \
                            and track['radarDistCamFrame'] > 0.1:
                        fill_color = [0, 255, 0]  # Green
                    else:
                        fill_color = [0, 0, 255]  # Blue

                    builder.primitive('/tracking_targets')\
                        .circle([x, y, z], .5)\
                        .style({'fill_color': fill_color})\
                        .id(track['trackId'])

                    text = f"[{track['classId'][0]}]{track['trackId']}"
                    builder.primitive('/tracking_id')\
                            .text(text)\
                            .position([x, y, z+.1])\
                            .id(track['trackId'])

                    self.track_history[track['trackId']] = track['radarDistCamFrame']
        except Exception as e:
            print('Crashed in draw tracking targets:', e)


    def _draw_camera_targets(self, camera_output, builder: xviz.XVIZBuilder):
        if camera_output is None:
            return
        try:
            for target in camera_output['targets']:
                (x, y, z) = self.get_object_xyz(
                    target,
                    'objectAngle',
                    'objectDistance',
                    radar_ob=False
                )
                if target['label'] == 'qrcode':
                    continue

                _phi = target['objectAngle']
                _dr = target['objectDistance']

                builder.primitive('/camera_targets')\
                    .circle([x, y, z], .5)\
                    .id(target['label'])

        except Exception as e:
            print('Crashed in draw camera targets:', e)


    def _draw_combine(self, builder: xviz.XVIZBuilder):
        if not self.tractor_state or not self.combine_states:
            return
        try:
            _, tractor_state = self.tractor_state[-1]
            tractor_heading = tractor_state['heading']  # degrees

            for combine_id, combine_state_tuple in self.combine_states.items():
                _, combine_state = combine_state_tuple
                combine_heading = combine_state['heading']  # degrees
                combine_speed = combine_state['speed']
                combine_relative_theta = (tractor_heading - combine_heading) * math.pi / 180

                gps_x, gps_y = transform_combine_to_local(combine_state, tractor_state, self.utm_zone)

                if combine_id == "combine":  # controlling combine
                    self.combine_x = gps_x
                    self.combine_y = gps_y
                    self.combine_relative_theta = combine_relative_theta

                gps_x -= self.tractor_gps_to_rear_axle

                combine_region = get_combine_region(
                    gps_x,
                    gps_y,
                    combine_relative_theta,
                    self.combine_dimensions['body_width'],
                    self.combine_dimensions['header_length'],
                    self.header_width + 1.0,
                    self.combine_dimensions['gps_to_header'],
                    self.combine_dimensions['gps_to_back'],
                )

                z = 0.5
                vertices = list(np.column_stack((
                    combine_region,
                    np.full(combine_region.shape[0], z)
                )).flatten())

                builder.primitive('/combine')\
                    .polyline(vertices)\
                    .id('combine')

                builder.primitive('/combine_speed')\
                    .text("C_speed: " + str(round(combine_speed, 3)))\
                    .position([self.combine_x-10., self.combine_y-10, 1.])\
                    .id('combine_speed')

        except Exception as e:
            print('Crashed in draw machine state:', e)


    def _draw_auger(self, builder: xviz.XVIZBuilder):
        if self.combine_x is None\
                or self.sync_status is None \
                or not self.sync_status['runningSync']:
            return
        try:
            combine_gps_x = self.combine_x - self.tractor_gps_to_rear_axle
            combine_gps_y = self.combine_y
            auger_region = get_auger_region(
                combine_gps_x,
                combine_gps_y,
                self.combine_relative_theta,
                self.combine_dimensions['body_width'],
                self.combine_dimensions['auger_length'],
                self.combine_dimensions['auger_width'],
                self.combine_dimensions['gps_to_auger'],
            )

            z = 0.5
            vertices = list(np.column_stack((
                auger_region,
                np.full(auger_region.shape[0], z)
            )).flatten())

            builder.primitive('/auger')\
                .polyline(vertices)\
                .id('auger')

        except Exception as e:
            print('Crashed in draw auger:', e)


    def _draw_predicted_paths(self, builder: xviz.XVIZBuilder):
        if not self.tractor_state:
            return
        try:
            _, tractor_state = self.tractor_state[-1]
            veh_speed = max(tractor_state['speed'], 0.447 * 1.0)

            sync_stop_threshold, waypoint_stop_threshold, \
                sync_slowdown_threshold, _waypoint_slowdown_threshold \
                = get_path_distances(veh_speed, self.global_config['safety'])

            wheel_angle = get_wheel_angle(
                tractor_state['curvature'],
                self.global_config['guidance']['wheel_base'],
            )

            # # Code to verify the actual curvature matches the yawRate/speed
            # temp_veh_speed = max(tractor_state['speed'], .01)
            # cT = (tractor_state['curvature'])
            # yaw_rate_rad = (tractor_state['yawRateDeg'] * math.pi/180.)
            # cE = (yaw_rate_rad / (temp_veh_speed))

            self._draw_predictive_polygons(veh_speed, wheel_angle,
                sync_stop_threshold, sync_slowdown_threshold, builder)
            self._draw_vision_polygons(veh_speed, wheel_angle,
                sync_stop_threshold, waypoint_stop_threshold, builder)

        except Exception as e:
            print('Crashed in draw predicted paths:', e)
            raise e


    def _draw_predictive_polygons(self, veh_speed, wheel_angle,
            stop_threshold, slowdown_threshold, builder: xviz.XVIZBuilder):
        if self.sync_status is None:
            sync_status = dict(runningSync=False, inSync=False, atSyncPoint=False)
        else:
            sync_status = self.sync_status
        try:
            stop_poly = get_path_poly(
                veh_speed,
                self.global_config['guidance']['wheel_base'],
                wheel_angle,
                self.global_config['safety']['path_widths']['narrow'],
                stop_threshold,
                0.,
                0.,
                0.,
            )
            slow_poly = get_path_poly(
                veh_speed,
                self.global_config['guidance']['wheel_base'],
                wheel_angle,
                self.global_config['safety']['path_widths']['narrow'],
                slowdown_threshold,
                0.,
                0.,
                0.,
            )

            predictive_polys = []

            if not sync_status['inSync']:
                predictive_polys.append(slow_poly)

            for poly in predictive_polys:
                builder.primitive('/predictive_polygons')\
                    .polyline(poly)\
                    .id('predictive_polygons')

        except Exception as e:
            print('Crashed in draw predictive polygons:', e)
            raise e


    def _draw_vision_polygons(self, veh_speed, wheel_angle, sync_stop_threshold,
                                waypoint_stop_threshold, builder: xviz.XVIZBuilder):
        if self.sync_status is None:
            sync_status = dict(runningSync=False, inSync=False, atSyncPoint=False)
        else:
            sync_status = self.sync_status
        try:
            sync_stop_poly = get_path_poly(
                veh_speed,
                self.global_config['guidance']['wheel_base'],
                wheel_angle,
                self.global_config['safety']['path_widths']['narrow'],
                sync_stop_threshold,
                0.,
                0.,
                0.,
            )
            waypoint_stop_poly = get_path_poly(
                veh_speed,
                self.global_config['guidance']['wheel_base'],
                wheel_angle,
                self.global_config['safety']['path_widths']['default'],
                waypoint_stop_threshold,
                0.,
                0.,
                0.,
            )

            vision_polys = [waypoint_stop_poly]

            clostest_point_ind, stop_poly = self.get_closest_point_actual_path_poly(builder)
            if stop_poly is not None:
                vision_polys = [stop_poly]


            if sync_status['runningSync']:
                vision_polys.append(sync_stop_poly)

            for poly in vision_polys:
                builder.primitive('/vision_polygons')\
                    .polyline(poly)\
                    .id('vision_polygons')

        except Exception as e:
            print('Crashed in draw predictive polygons:', e)
            raise e


    def _draw_control_signal(self, builder: xviz.XVIZBuilder):
        if self.control_signal is None:
            return
        try:
            speed = self.control_signal['setSpeed']
            curvature = self.control_signal['commandCurvature']
            wheel_angle = get_wheel_angle(
                curvature, self.global_config['guidance']['wheel_base'])
            time_horizon = 10.0
            U = (speed, wheel_angle)
            X0 = (0., 0., 0.)
            C = dict(
                wheel_base=self.global_config['guidance']['wheel_base'],
                machine_width=1.,
            )

            path, _, _ = predict_path(X0, U, C, time_horizon)

            z = 1.1
            path[:, 2] = z
            vertices = list(path.flatten())

            builder.primitive('/control_signal')\
                .polyline(vertices)\
                .id('control_signal')

            builder.primitive('/set_speed')\
                .text("set_speed: " + str(round(speed, 3)))\
                .position([-self.tractor_gps_to_rear_axle-20., 10., 1.])\
                .id('set speed')

            builder.primitive('/set_wheel_angle')\
                .text("set_wheel_angle: " + str(round(wheel_angle, 3)))\
                .position([-self.tractor_gps_to_rear_axle+20., 10., 1.])\
                .id('set wheel angle')

        except Exception as e:
            print('Crashed in draw control signal:', e)


    def get_closest_point_actual_path_poly(self, builder: xviz.XVIZBuilder):

        ready_to_draw = self.planned_path is not None and \
            self.index > 0 and \
            self.index < len(self.collector_instances)-1 and \
            len(self.tractor_state) > 0 and \
            len(self.planned_path) > 0

        clostest_point_ind = 0
        stop_poly = None
        if ready_to_draw:
            _, tractor_state = self.tractor_state[-1]
            xy_array = utm_array_to_local(tractor_state, self.utm_zone, self.planned_path)
            clostest_point_ind = closest_node([0, 0], xy_array)

            xy_array = np.append(xy_array, np.ones((xy_array.shape[0],1)), axis=1)
            z = 1.0
            if is_close_to_curve(builder, xy_array, i=clostest_point_ind, LOOK_AHEAD_METERS=10., LOOK_BEHIND_METERS=5.):
                machine_width = self.global_config['safety']['path_widths']['default']
                stop_poly = get_actual_path_poly(data=xy_array, i=clostest_point_ind, machine_width=machine_width, LOOK_AHEAD_METERS=10.)


        return clostest_point_ind, stop_poly

    def _draw_planned_path(self, builder: xviz.XVIZBuilder):
        if self.planned_path is None:
            return
        try:
            _, tractor_state = self.tractor_state[-1]
            xy_array = utm_array_to_local(tractor_state, self.utm_zone, self.planned_path)
            z = 1.0

            assert len(self.planned_path) == self.count_uniques(self.planned_path)

            clostest_point_ind, stop_poly = self.get_closest_point_actual_path_poly(builder)

            vertices = list(np.column_stack(
                (xy_array, np.full(xy_array.shape[0], z))
            ).flatten())

            builder.primitive('/planned_path')\
                .polyline(vertices)\
                .id('planned_path')

            builder.primitive('/closest_point')\
                .circle([xy_array[clostest_point_ind, 0], xy_array[clostest_point_ind, 1], 5], .3)\
                .id('closest point')

        except Exception as e:
            print('Crashed in draw planned path:', e)
            raise e


    def _draw_field_definition(self, builder: xviz.XVIZBuilder):
        if self.field_definition is None or self.tractor_easting is None:
            return
        try:
            poly = []
            if self.field_definition['type'] == 'MultiPolygon':
                for polygons in self.field_definition['coordinates']:
                    for polygon in polygons:
                        poly.append(np.array(polygon))

            else:
                for polygon in self.field_definition['coordinates']:
                    poly.append(np.array(polygon))

            for p in poly:
                utm_coords = np.array(p)
                utm_coords[:, 0] -= self.tractor_easting
                utm_coords[:, 1] -= self.tractor_northing

                z = 1.0
                vertices = list(np.column_stack(
                    (utm_coords, np.full(utm_coords.shape[0], z))
                ).flatten())

                builder.primitive('/field_definition')\
                    .polyline(vertices)\
                    .id('field_definition')

        except Exception as e:
            print('Crashed in draw field definition:', e)


    def _draw_sync_status(self, builder: xviz.XVIZBuilder):
        if self.sync_status is None:
            return
        try:
            if self.sync_status['atSyncPoint']:
                text = "at sync point"
            elif self.sync_status['inSync']:
                text = "in sync"
            elif self.sync_status['runningSync']:
                text = "running sync"
            else:
                text = ""

            builder.primitive('/sync_status')\
                .text(text)\
                .position([-self.tractor_gps_to_rear_axle-3., 0., 1.])\
                .id('sync status')

        except Exception as e:
            print('Crashed in draw sync status:', e)


    def _draw_sync_params(self, builder: xviz.XVIZBuilder):
        if self.sync_params is None \
                or self.combine_x is None:
            return

        try:
            sync_x_rel_combine = self.sync_params['sync_point'][0] + self.sync_params['sync_dx']
            sync_y_rel_combine = self.sync_params['sync_point'][1] + self.sync_params['sync_dy']
            sync_x = self.combine_x \
                + sync_x_rel_combine * math.cos(self.combine_relative_theta) \
                - sync_y_rel_combine * math.sin(self.combine_relative_theta)
            sync_y = self.combine_y \
                + sync_x_rel_combine * math.sin(self.combine_relative_theta) \
                + sync_y_rel_combine * math.cos(self.combine_relative_theta)
            z = 2.0

            builder.primitive('/sync_point')\
                .circle([sync_x, sync_y, z], .3)\
                .id('sync point')

            if self.sync_params['breadcrumbs']:
                _, tractor_state = self.tractor_state[-1]
                breadcrumbs_xy = lonlat_array_to_local(
                    tractor_state,
                    self.utm_zone,
                    np.array(self.sync_params['breadcrumbs'])
                )
                vertices = list(np.column_stack(
                    (breadcrumbs_xy, np.full(breadcrumbs_xy.shape[0], z-.1))
                ).flatten())

                builder.primitive('/breadcrumbs')\
                    .polyline(vertices)\
                    .id('breadcrumbs')

        except Exception as e:
            print('Crashed in draw sync params:', e)


    def update_machine_state(self, machine_state):
        if not self.utm_zone:
            # only need to set it once
            self.utm_zone = machine_state['opState']['refUtmZone']

        self.header_width = machine_state['opState']['machineWidth']
        vehicle_states = machine_state['vehicleStates']
        if vehicle_states:
            for vehicle, state in vehicle_states.items():
                if vehicle == 'tractor':
                    self.tractor_state.append((self.index, state))
                else:
                    self.combine_states[vehicle] = (self.index, state)


    def _is_vehicle_state_old(self, vehicle_state_tuple):
        last_updated_index, _ = vehicle_state_tuple
        return self.index - last_updated_index > 5


    def get_object_xyz(self, ob, angle_key, dist_key, radar_ob=False):
        x = math.cos(ob[angle_key]) * ob[dist_key]
        y = math.sin(ob[angle_key]) * ob[dist_key]
        z = 1.5

        if radar_ob:
            x += self.cab_to_nose

        return (x, y, z)


    def get_object_xyz_primitive(self, radial_dist, angle_radians):
        x = math.cos(angle_radians) * radial_dist
        y = math.sin(angle_radians) * radial_dist
        z = 1.0

        return (x, y, z)
