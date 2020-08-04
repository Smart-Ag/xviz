import math
import time
import shutil
import cv2
import numpy as np
from collections import deque
from pathlib import Path
from google.protobuf.json_format import MessageToDict

import xviz
import xviz.builder as xbuilder

def prepare_parent_import(N):
    import sys
    # N = number of parent directories up that you want to import from, cwd of file would be N=0
    top_dir = Path(__file__).resolve().parents[N]
    sys.path.append(str(top_dir / 'Proto-Files'))

prepare_parent_import(3)
from protobuf_APIs import collector_pb2, falconeye_pb2, radar_pb2, camera_pb2


DEG_1_AS_RAD = math.pi / 180
DEG_90_AS_RAD = 90 * DEG_1_AS_RAD


class RadarFilter:

    def __init__(self, qfilter_enabled=True, queue_size=12, consecutive_min=7,
                    pexist_min=0.8, d_bpower_min=-10, phi_sdv_max=0.1, nan_threshold=0.5):
        self.qfilter_enabled = qfilter_enabled
        self.queue_size = queue_size
        self.consecutive_min = consecutive_min
        self.pexist_min = pexist_min
        self.d_bpower_min = d_bpower_min
        self.phi_sdv_max = phi_sdv_max
        self.nan_threshold = nan_threshold # maximum percent of queue that can be nan before it is automatically evaluated as an invalid target

        self.target_queues = {}


    def is_valid_target(self, target_id, target):
        if self.qfilter_enabled:
            self.make_target_queue_if_nonexistent(target_id)
            self.update_queues(target_id, target)

            if self.is_default_target(target):
                return False

            return self.queue_filter(target_id)

        # use passive filter
        if self.is_default_target(target):
            return False

        return self.passive_filter(target)


    def passive_filter(self, target):
        ''' Determines if the target is valid or noise based on simple value checks.
            Returns True if the target is valid.
        '''
        if target['consecutive'] < self.consecutive_min \
            or target['pexist'] < self.pexist_min \
            or target['d_bpower'] <= self.d_bpower_min \
            or target['phi_sdv'] >= self.phi_sdv_max:
            return False
        return True
            

    def queue_filter(self, target_id):
        ''' Determines if the target is valid or noise based on a given method.
            Returns True if the target is valid.
        '''
        if np.isnan(self.target_queues[target_id]['consecutive_queue']).sum() / self.queue_size > self.nan_threshold \
            or np.nanmean(self.target_queues[target_id]['consecutive_queue']) < self.consecutive_min \
            or np.nanmean(self.target_queues[target_id]['pexist_queue']) < self.pexist_min \
            or np.nanmean(self.target_queues[target_id]['d_bpower_queue']) <= self.d_bpower_min \
            or np.nanmean(self.target_queues[target_id]['phi_sdv_queue']) >= self.phi_sdv_max:
            return False
        return True

        
    def is_default_target(self, target):
        ''' Determines if there are measurments corresponding to the given target
            or if it is just a default message.
            Returns True if the target is a default message.
        '''
        if target['consecutive'] < 1:
            return True
        return False


    def update_queues(self, target_id, target):
        if self.is_default_target(target):
            self.target_queues[target_id]['consecutive_queue'].append(np.nan)
            self.target_queues[target_id]['pexist_queue'].append(np.nan)
            self.target_queues[target_id]['d_bpower_queue'].append(np.nan)
            self.target_queues[target_id]['phi_sdv_queue'].append(np.nan)
        else:
            self.target_queues[target_id]['consecutive_queue'].append(target['consecutive'])
            self.target_queues[target_id]['pexist_queue'].append(target['pexist'])
            self.target_queues[target_id]['d_bpower_queue'].append(target['d_bpower'])
            self.target_queues[target_id]['phi_sdv_queue'].append(target['phi_sdv'])


    def make_target_queue_if_nonexistent(self, target_id):
        if target_id not in self.target_queues:
            self.target_queues[target_id] = {}
            self.target_queues[target_id]['consecutive_queue'] = deque(maxlen=self.queue_size)
            self.target_queues[target_id]['pexist_queue'] = deque(maxlen=self.queue_size)
            self.target_queues[target_id]['d_bpower_queue'] = deque(maxlen=self.queue_size)
            self.target_queues[target_id]['phi_sdv_queue'] = deque(maxlen=self.queue_size)


class CollectorScenario:
    def __init__(self, live=True, radius=30, duration=10, speed=10):
        self._timestamp = time.time()
        self._radius = radius
        self._duration = duration
        self._speed = speed
        self._live = live
        self._metadata = None
        self.index = 0
        self.id_tracks = {}
        self.id_last = 1
        self.data = []

        tar_file = '/home/raven.ravenind.net/r103943/Desktop/collector/v2020-25-0-25ed12058f204e60ab6bf655e1d95640-nodetection-primary-forward-57-smartag-autocart-1596479215515-3668.tar'
        extract_dir = '/home/raven.ravenind.net/r103943/Desktop/collector/extracted'
        tar_file = Path(tar_file)
        extract_dir = Path(extract_dir)

        if not tar_file.is_file():
            print('tar file does not exit')
        if not extract_dir.is_dir():
            extract_dir.mkdir(parents=True)

        # if there are no txt files in the directory, the tar needs to be unpacked
        if not list(extract_dir.glob('*.txt')):
            shutil.unpack_archive(str(tar_file), str(extract_dir))

        self.perception_instances = sorted(extract_dir.glob('*.txt'))

        self.radar_filter = RadarFilter()


    def get_metadata(self):
        if not self._metadata:
            builder = xviz.XVIZMetadataBuilder()
            builder.stream("/vehicle_pose").category(xviz.CATEGORY.POSE)
            builder.stream("/radar_targets")\
                .coordinate(xviz.COORDINATE_TYPES.IDENTITY)\
                .stream_style({'fill_color': [200, 0, 70, 128]})\
                .category(xviz.CATEGORY.PRIMITIVE)\
                .type(xviz.PRIMITIVE_TYPES.CIRCLE)
            builder.stream("/tracking_targets")\
                .coordinate(xviz.COORDINATE_TYPES.IDENTITY)\
                .stream_style({'fill_color': [200, 0, 70, 128]})\
                .category(xviz.CATEGORY.PRIMITIVE)\
                .type(xviz.PRIMITIVE_TYPES.CIRCLE)
            builder.stream("/camera_targets")\
                .coordinate(xviz.COORDINATE_TYPES.IDENTITY)\
                .stream_style({'fill_color': [200, 0, 70, 128]})\
                .category(xviz.CATEGORY.PRIMITIVE)\
                .type(xviz.PRIMITIVE_TYPES.CIRCLE)
            
            builder.stream("/measuring_circles")\
                .coordinate(xviz.COORDINATE_TYPES.IDENTITY)\
                .stream_style({
                    'stroked': True,
                    'stroke_width': 1,
                    'stroke_color': [0, 255, 0, 128],
                    #'stroke_width_min_pixels': 10
                })\
                .category(xviz.CATEGORY.PRIMITIVE)\
                .type(xviz.PRIMITIVE_TYPES.CIRCLE)

            
            builder.stream("/measuring_circles_lbl")\
                .coordinate(xviz.COORDINATE_TYPES.IDENTITY)\
                .stream_style({
                    'fill_color': [0, 0, 0]
                })\
                .category(xviz.CATEGORY.PRIMITIVE)\
                .type(xviz.PRIMITIVE_TYPES.TEXT)

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
            self._draw_measuring_circles(builder, timestamp)
            self._draw_perception_outputs(builder, timestamp)
            data = builder.get_message()

            return {
                'type': 'xviz/state_update',
                'data': data.to_object()
            }
        except Exception as e:
            print("Crashed in get_message:", e)


    def _draw_measuring_circles(self, builder: xviz.XVIZBuilder, timestamp):

        builder.primitive('/measuring_circles_lbl').text("30").position([30, 0, 0]).id('30lb')
        builder.primitive('/measuring_circles_lbl').text("25").position([25, 0, 0]).id('25lb')
        builder.primitive('/measuring_circles_lbl').text("20").position([20, 0, 0]).id('20lb')
        builder.primitive('/measuring_circles_lbl').text("15").position([15, 0, 0]).id('15lb')
        builder.primitive('/measuring_circles_lbl').text("10").position([10, 0, 0]).id('10lb')
        builder.primitive('/measuring_circles_lbl').text("5").position([5, 0, 0]).id('5lb')

        builder.primitive('/measuring_circles').circle([0, 0, 0], 30).id('30')
        builder.primitive('/measuring_circles').circle([0, 0, 0], 25).id('25')
        builder.primitive('/measuring_circles').circle([0, 0, 0], 20).id('20')
        builder.primitive('/measuring_circles').circle([0, 0, 0], 15).id('15')
        builder.primitive('/measuring_circles').circle([0, 0, 0], 10).id('10')
        builder.primitive('/measuring_circles').circle([0, 0, 0], 5).id('5')

    
    def _draw_perception_outputs(self, builder: xviz.XVIZBuilder, timestamp):
        try:
            print("Drawing perception instance")
            builder.pose()\
                .timestamp(timestamp)

            if self.index == len(self.perception_instances):
                self.index = 0

            perception_instance = self.perception_instances[self.index]

            collector_proto_msg = self.deserialize_collector_proto_msg(perception_instance)
            img = self.extract_image(collector_proto_msg.frame)
            camera_output, radar_output, tracking_output = self.extract_proto_msgs(collector_proto_msg)

            if radar_output:
                self._draw_radar_targets(radar_output, builder)
            if tracking_output:
                self._draw_tracking_targets(tracking_output, builder)
            if camera_output:
                self._draw_camera_targets(camera_output, builder)

            self.index += 1
        
        except Exception as e:
            print('Crashed in draw perception outputs: ', e)

    
    def _draw_radar_targets(self, radar_output, builder: xviz.XVIZBuilder):
        try:
            for target in radar_output['targets'].values():
                if 'dr' in target:
                    (x, y, z) = self.get_target_xyz(target)
                    if self.radar_filter.is_valid_target(target['target_id'], target):
                        #Red
                        fill_color = [255, 0, 0]
                    else:
                        #Blue
                        fill_color = [0, 0, 255]
                    builder.primitive('/radar_targets').circle([x, y, z], .5)\
                        .style({'fill_color': fill_color})\
                        .id(str(target['target_id']))
        except Exception as e:
            print('Crashed in draw radar targets: ', e)

    
    def _draw_tracking_targets(self, camera_output):
        pass


    def _draw_camera_targets(self, camera_output):
        pass
    

    def get_target_xyz(self, target):
        x = math.cos(target['phi']) * target['dr']
        y = math.sin(target['phi']) * target['dr']
        z = .1

        return (x, y, z)


    def deserialize_collector_proto_msg(self, file_path):
        collector_proto_msg = collector_pb2.CollectorOutput()
        collector_proto_msg.ParseFromString(Path(file_path).read_bytes())
        return collector_proto_msg


    def extract_image(self, img_bytes):
        encoded_img = np.frombuffer(img_bytes, dtype=np.uint8) # decode bytes
        decimg = cv2.imdecode(encoded_img, 1) # uncompress image
        return decimg

    
    def extract_proto_msgs(self, collector_proto_msg):
        camera_output = camera_pb2.CameraOutput()
        camera_output.ParseFromString(collector_proto_msg.camera_output)
        camera_output = MessageToDict(camera_output)

        radar_output = radar_pb2.RadarOutput()
        radar_output.ParseFromString(collector_proto_msg.radar_output)
        radar_output = MessageToDict(radar_output)

        tracking_output = falconeye_pb2.TrackingOutput()
        tracking_output.ParseFromString(collector_proto_msg.tracking_output)
        tracking_output = MessageToDict(tracking_output)

        return camera_output, radar_output, tracking_output