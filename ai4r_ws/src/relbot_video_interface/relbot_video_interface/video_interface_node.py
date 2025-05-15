#!/usr/bin/env python3
# test the link
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import gi
import numpy as np
import cv2
from ultralytics import YOLO
from ultralytics.utils import ROOT
from collections import defaultdict

gi.require_version('Gst', '1.0')
from gi.repository import Gst


class VideoInterfaceNode(Node):
    def __init__(self):
        super().__init__('video_interface')
        # Publisher: sends object position to the RELBot
        # Topic `/object_position` is watched by the robot controller for actuation
        self.position_pub = self.create_publisher(Point, '/object_position', 10)

        # Declare GStreamer pipeline as a parameter for flexibility
        self.declare_parameter('gst_pipeline', (
            'udpsrc port=5000 caps="application/x-rtp,media=video,'
            'encoding-name=H264,payload=96" ! '
            'rtph264depay ! avdec_h264 ! videoconvert ! '
            'video/x-raw,format=RGB ! appsink name=sink'
        ))
        pipeline_str = self.get_parameter('gst_pipeline').value

        # Initialize GStreamer and build pipeline
        Gst.init(None)
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.sink = self.pipeline.get_by_name('sink')
        # Drop late frames to ensure real-time processing
        self.sink.set_property('drop', True)
        self.sink.set_property('max-buffers', 1)
        self.pipeline.set_state(Gst.State.PLAYING)

        # Timer: fires at ~30Hz to pull frames and publish positions
        # The period (1/30) sets how often on_timer() is called
        self.timer = self.create_timer(1.0 / 30.0, self.on_timer)
        self.get_logger().info('VideoInterfaceNode initialized, streaming at 30Hz')
        self.track_history = defaultdict(lambda: [])
        self.model = YOLO("./weights.pt")

    def on_timer(self):
        # Pull the latest frame from the GStreamer appsink
        sample = self.sink.emit('pull-sample')
        if not sample:
            # No new frame available
            return

        buf = sample.get_buffer()
        caps = sample.get_caps()
        width = caps.get_structure(0).get_value('width')
        height = caps.get_structure(0).get_value('height')
        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if not ok:
            # Failed to map buffer data
            return

        # Convert raw buffer to numpy array [height, width, channels]
        frame = np.frombuffer(mapinfo.data, np.uint8).reshape(height, width, 3)
        buf.unmap(mapinfo)

        # Run the tracker
        #results = self.model(frame)

        # tracker_config = str(ROOT / 'trackers/cfg/bytetrack.yaml')
        results = self.model.track(frame, tracker="bytetrack.yaml", persist=True, conf=0.4, iou=0.1)
        annotated_frame = results[0].plot()

        #
        # # Draw tracking results
        #if results and results[0].boxes.id is not None:
        #     boxes = results[0].boxes.xyxy.cpu().numpy()  # [x1, y1, x2, y2]
        #     ids = results[0].boxes.id.cpu().numpy()  # track IDs
        #
        #     for box, obj_id in zip(boxes, ids):
        #         x1, y1, x2, y2 = map(int, box)
        #         cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        #         cv2.putText(frame, f'ID: {int(obj_id)}', (x1, y1 - 10),
        #                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        #
        # # Show the frame
        #cv2.imshow('Tracker Visualization', frame)
        #cv2.waitKey(1)  # Needed for OpenCV window refresh

        # for r in results:
        # #     best_person = None
        # #     max_area = 0
        # #
        #      for box in r.boxes:
        #          cls = int(box.cls[0])
        # #         print('Class found: ', self.model.names[cls])
        #          if self.model.names[cls] == 'Hardhat':
        #             x1, y1, x2, y2 = map(int, box.xyxy[0])
        #             msg = Point()
        #             msg.x = float((x1 + x2) / 2)
        #             msg.y = 0.0
        #             msg.z = 10001.0  # Used to infer distance
        #
        #             self.position_pub.publish(msg)
        #             self.get_logger().debug(f'Published: ({msg.x:.1f}, area={msg.z:.1f})')


        #             x1, y1, x2, y2 = map(int, box.xyxy[0])
        #             area = (x2 - x1) * (y2 - y1)
        #
        #             if area > max_area:
        #                 max_area = area
        #                 best_person = (x1, y1, x2, y2)
        #
        #     if best_person:
        #         x1, y1, x2, y2 = best_person
        #         center_x = (x1 + x2) / 2
        #
        #         msg = Point()
        #         msg.x = float(center_x)
        #         msg.y = 0.0
        #         msg.z = 10001.0  # Used to infer distance
        #
        #         self.position_pub.publish(msg)
        #         self.get_logger().debug(f'Published: ({msg.x:.1f}, area={msg.z:.1f})')
        #
        #         # Optional: draw bounding box on stream
        #         frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        #         cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
        #         cv2.circle(frame_bgr, (int(center_x), y2), 5, (0, 0, 255), -1)

        if results[0].boxes and results[0].boxes.id is not None:
            boxes = results[0].boxes.xywh.cpu()
            track_ids = results[0].boxes.id.int().cpu().tolist()
            for box, track_id in zip(boxes, track_ids):
                x, y, w, h = box
                track = self.track_history[track_id]
                track.append((float(x), float(y)))
                if len(track) > 100:
                    track.pop(0)
                points = np.hstack(track).astype(np.int32).reshape((-1,1,2))
                cv2.polylines(annotated_frame, [points], isClosed=False, color=(230,230,230), thickness=2)

        frame_bgr = cv2.cvtColor(annotated_frame, cv2.COLOR_RGB2BGR)
        cv2.imshow('Input Stream', frame_bgr)
        cv2.waitKey(1)

    def destroy_node(self):
        # Cleanup GStreamer resources on shutdown
        self.pipeline.set_state(Gst.State.NULL)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoInterfaceNode()
    try:
        rclpy.spin(node)  # Keep node alive, invoking on_timer periodically
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
