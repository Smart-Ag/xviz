# Generate a scenario with multiple objects in XVIZ
"""
This module provides a example scenario where a vehicle drives along a circle.
"""

import time
import math
from random import randint
import cairo

import io as pyio

import xviz_avs as xviz
import xviz_avs.builder as builder
from xviz_avs.builder import XVIZUIBuilder

import xviz_avs.io as io

# This demonstrates using the XVIZBuilder to create XVIZ data.
class SimulationGenerator:
    def drawGrid(self, builder, side=10, count=5):
        lineMax = side*count
        lineMin = -lineMax
        for r in range(-count, count+1):
            pt = r * side
            builder.primitive('/grid')\
                    .polyline([lineMin, pt, -0.1, lineMax, pt, -0.1])
            builder.primitive('/grid')\
                    .polyline([pt, lineMin, -0.1, pt, lineMax, -0.1])

    def getPoint(self, entity, delta):
        x = entity["start"][0] + (delta * entity["direction"][0])
        y = entity["start"][1] + (delta * entity["direction"][1])
        return [x, y, 0]

    def getPolygon(self, entity, delta):
        pt = self.getPoint(entity, delta)
        return [pt[0]-1.5, pt[1]-1.5, 0,
                pt[0]-1.5, pt[1]+1.5, 0,
                pt[0]+1.5, pt[1]+1.5, 0,
                pt[0]+1.5, pt[1]-1.5, 0]

    def draw_vehicle(self, entity, start, delta, builder):
        pt = self.getPoint(entity, delta)
        builder.pose()\
            .timestamp(start + delta)\
            .orientation(0, 0, math.pi/2)\
            .position(pt[0], pt[1], pt[2])

        builder.time_series("/vehicle/metric")\
            .timestamp(start + delta)\
            .value(randint(10,15))
        builder.time_series("/vehicle/metric2")\
            .timestamp(start + delta)\
            .value(randint(8,12))

        lower = -2*math.pi
        step = (4*math.pi)/200
        independent = [lower + i*step for i in range(1, 200)]
        builder.variable("/vehicle/plot")\
            .values(independent)
        builder.variable("/plot1")\
            .values([math.cos(theta) for theta in independent])
        builder.variable("/plot2")\
            .values([math.sin(theta) for theta in independent])

    def draw_table(self, delta, builder):
        cols = [
            {'display_text': 'string', 'type': 'STRING'},
            {'display_text': 'int32', 'type': 'INT32'},
            {'display_text': 'bool', 'type': 'BOOLEAN'},
            {'display_text': 'double', 'type': 'DOUBLE'}
        ]
        table = builder.ui_primitives("/table/1").treetable(cols)

        for i in range(10):
            table.row(None, ['delta {}'.format(delta), i, i%2, delta * math.pi])

    def draw_treetable(self, delta, builder):
        cols = [
            {'display_text': 'string', 'type': 'STRING'},
            {'display_text': 'int32', 'type': 'INT32'},
            {'display_text': 'bool', 'type': 'BOOLEAN'},
            {'display_text': 'double', 'type': 'DOUBLE'}
        ]
        table = builder.ui_primitives("/treetable/1").treetable(cols)
        row = table.row(1, ['delta {}'.format(delta*2), 0, False, delta * 2 * math.pi])
        for i in range(2, 10):
            new_row = row.child(i, ['delta {}'.format(delta*2), i, i%2, delta * 2 * math.pi])
            if i%3 is 0:
                row = new_row

    def draw_video(self, delta, builder):
        WIDTH, HEIGHT = 100, 100

        surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, WIDTH, HEIGHT)
        ctx = cairo.Context(surface)

        ctx.scale(WIDTH, HEIGHT)  # Normalizing the canvas
        ctx.rectangle(0, 0, 1, 1)  # Rectangle(x0, y0, x1, y1)
        ctx.fill()

        ctx.translate(0.5, 0.5)  # Changing the current transformation matrix

        ctx.move_to(0.4, 0)
        # Arc(cx, cy, radius, start_angle, stop_angle)
        ctx.arc(0, 0, 0.4, 0, delta* math.pi)
        ctx.line_to(0, 0)  # Line to (x,y)
        ctx.close_path()

        ctx.set_source_rgb(0.3, 0.2, 0.5)  # Solid color
        ctx.set_line_width(0.02)
        ctx.stroke()

        target = pyio.BytesIO()
        surface.write_to_png(target)  # Output to PNG
        target.seek(0)
        builder.primitive('/camera/1')\
                .image(target.read())

    def draw_object(self, name, entity, delta, builder):
        builder.primitive("/objects/shape")\
            .polygon(self.getPolygon(entity, delta))\
            .id(name)

        pt = self.getPoint(entity, delta)
        builder.primitive("/objects/text")\
            .text(name)\
            .position([pt[0]+1, pt[1]+1, 1.5])\
            .id(name)

        builder.primitive("/objects/vector")\
            .polyline([pt[0], pt[1], pt[2], pt[0] + entity["direction"][0], pt[1] + entity["direction"][1], 0])\
            .id(name)

        builder.primitive("/objects/path")\
          .polyline([p for v in entity["path"] for p in [v[0], v[1], 0]])\
          .id(name)\
          .style({'stroke_color': [0, 0, 100 + int(name) * 5, 128]})

        builder.primitive("/objects/traj")\
          .polyline([entity["start"][0], entity["start"][1], 0, entity["start"][0] + entity["direction"][0], entity["start"][1] + entity["direction"][1], 0])\
          .id(name)

    def _generate_xviz(self, data, start, end):
        store = io.MemorySource()
        writer = io.XVIZProtobufWriter(store)

        meta = self.get_metadata(start, end)
        writer.write_message(meta)

        step = 0.1 # 10hz
        current = start
        iteration = 0

        while current < end:
            builder = xviz.XVIZBuilder(metadata=meta)
            self.drawGrid(builder, count=20)

            delta = current - start
            self.draw_vehicle(data["ego"], start, delta, builder)
            self.draw_video(delta, builder)
            # self.draw_table(delta, builder)
            self.draw_treetable(delta, builder)

            for i in range(len(data["objects"])):
                obj = data["objects"][i]

                self.draw_object("{}".format(i), obj, iteration * step, builder)

                if iteration == 10:
                    obj["start"] = [obj["start"][0] + obj["direction"][0], obj["start"][1] + obj["direction"][1]]
                    obj["direction"] = [obj["direction"][0] + randint(-4, 4), obj["direction"][1] + randint(-4, 4)]
                    obj["path"].append(obj["start"])

            writer.write_message(builder.get_message())
            current += step
            iteration += 1
            if iteration > 10:
                iteration = 0

        return store

    def generate(self, count, duration):
        simulation = {
            "ego": {
                "start": [0, 0],
                "direction": [0, 4]
            },
            "objects": []
        }

        for i in range(0, count):
            startPt = [randint(-100, 100), randint(-100, 100)]
            simulation["objects"].append({
                "start": startPt,
                "direction": [randint(-10, 10), randint(-10, 10)],
                "path": [startPt]
            })

        start = time.time()
        return self._generate_xviz(simulation, start, start+duration)

    def get_metadata(self, start, end):
        builder = xviz.XVIZMetadataBuilder()
        builder.start_time(start)
        builder.end_time(end)

        ui_builder = XVIZUIBuilder()
        p = ui_builder.panel('metrics')
        p.child(ui_builder.metric(["/vehicle/metric", "/vehicle/metric2"], title="Test Metric", description="Test Description"))
        p.child(ui_builder.treetable("/treetable/1", title="TreeTable"))
        p.child(ui_builder.table("/table/1", title="Table"))

        p2 = ui_builder.panel('plot')
        p2.child(ui_builder.plot("/vehicle/plot", ["/plot1", "/plot2"], title="Test plot", description="Test Description"))
        p2.child(ui_builder.video(["/camera/1"]))

        ui_builder.child(p)
        ui_builder.child(p2)
        builder.ui(ui_builder)

        builder.stream("/vehicle_pose").category(xviz.CATEGORY.POSE)
        builder.stream("/vehicle/metric").category(xviz.CATEGORY.TIME_SERIES).type(xviz.SCALAR_TYPE.FLOAT)
        builder.stream("/vehicle/metric2").category(xviz.CATEGORY.TIME_SERIES).type(xviz.SCALAR_TYPE.FLOAT)
        builder.stream("/vehicle/plot").category(xviz.CATEGORY.VARIABLE).type(xviz.SCALAR_TYPE.FLOAT)
        builder.stream("/plot1").category(xviz.CATEGORY.VARIABLE).type(xviz.SCALAR_TYPE.FLOAT)
        builder.stream("/plot2").category(xviz.CATEGORY.VARIABLE).type(xviz.SCALAR_TYPE.FLOAT)

        builder.stream("/table/1")\
            .category(xviz.CATEGORY.PRIMITIVE)\
            .type(xviz.UIPRIMITIVE_TYPES.TREETABLE)
        builder.stream("/treetable/1")\
            .category(xviz.CATEGORY.PRIMITIVE)\
            .type(xviz.UIPRIMITIVE_TYPES.TREETABLE)
        builder.stream("/camera/1")\
            .category(xviz.CATEGORY.PRIMITIVE)\
            .type(xviz.PRIMITIVE_TYPES.IMAGE)
        builder.stream("/ground")\
            .coordinate(xviz.COORDINATE_TYPES.IDENTITY)\
            .stream_style({'filled': False, "stroked": True, "stroke_color": [30, 30, 30, 128]})\
            .category(xviz.CATEGORY.PRIMITIVE)\
            .type(xviz.PRIMITIVE_TYPES.CIRCLE)
        builder.stream("/objects/shape")\
            .coordinate(xviz.COORDINATE_TYPES.IDENTITY)\
            .stream_style({'fill_color': [200, 0, 70, 128], "extruded": True, "height": 1.0})\
            .category(xviz.CATEGORY.PRIMITIVE)\
            .type(xviz.PRIMITIVE_TYPES.CIRCLE)
        builder.stream("/objects/text")\
            .coordinate(xviz.COORDINATE_TYPES.IDENTITY)\
            .category(xviz.CATEGORY.PRIMITIVE)\
            .stream_style({'text_size': 24, 'text_baseline': 'BOTTOM', 'fill_color': [0, 0, 0, 255]})\
            .type(xviz.PRIMITIVE_TYPES.TEXT)
        builder.stream("/objects/path")\
            .coordinate(xviz.COORDINATE_TYPES.IDENTITY)\
            .category(xviz.CATEGORY.PRIMITIVE)\
            .type(xviz.PRIMITIVE_TYPES.POLYLINE)\
            .stream_style({'stroke_color': [0, 200, 40, 128], 'stroke_width': 2.0})
        builder.stream("/objects/traj")\
            .coordinate(xviz.COORDINATE_TYPES.IDENTITY)\
            .category(xviz.CATEGORY.PRIMITIVE)\
            .type(xviz.PRIMITIVE_TYPES.POLYLINE)\
            .stream_style({'stroke_color': [0, 0, 150, 128], 'stroke_width': 1.0})
        builder.stream("/objects/vector")\
            .coordinate(xviz.COORDINATE_TYPES.IDENTITY)\
            .category(xviz.CATEGORY.PRIMITIVE)\
            .type(xviz.PRIMITIVE_TYPES.POLYLINE)\
            .stream_style({'stroke_color': [200, 0, 0, 128], 'stroke_width': 0.5})
        builder.stream("/grid")\
            .coordinate(xviz.COORDINATE_TYPES.IDENTITY)\
            .category(xviz.CATEGORY.PRIMITIVE)\
            .type(xviz.PRIMITIVE_TYPES.POLYLINE)\
            .stream_style({'stroke_color': [50, 100, 25, 128], 'stroke_width': 0.4})

        return builder.get_message()