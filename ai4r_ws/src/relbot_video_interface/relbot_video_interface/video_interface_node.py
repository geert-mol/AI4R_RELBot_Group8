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
import torch

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
        
        #model selection
        #self.model = YOLO("./weights.pt")
        self.model = YOLO("yolov8n.pt")
        
        #used to switch between use of MIDAS model
        self.SIDE_use = True
        
        if self.SIDE_use:
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            #SIDE model MIDAS [not sure if it works offline]
            self.midas = torch.hub.load("intel-isl/MiDaS", "MiDaS_small") 
            self.midas.eval()
            self.midas.to(self.device)
            
            midastransforms=torch.hub.load("intel-isl/MiDaS", "transforms")
            self.transform=midastransforms.small_transform
        
        #Used for tracking single target
        self.target_id=None
        self.found_target=False
        self.target_cp=Point()
        
        #image_centerpoint for robot to stop moving
        self.image_cp=Point()
        self.image_cp.x=180.0  #idk what the exact value is should be updated.
        self.image_cp.z=10001.0
    
        
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

        #Yolo Model with ByteTrack tracker
        # # tracker_config = str(ROOT / 'trackers/cfg/bytetrack.yaml')
        # results = self.model.track(frame, tracker="bytetrack.yaml", persist=True, conf=0.4, iou=0.1)
        results = self.model.track(frame, tracker="bytetrack.yaml", persist=True, conf=0.4, iou=0.1,classes=[0]) #Should only track a people
        annotated_frame = results[0].plot()

        #SIDE depthmap
        if self.SIDE_use:
            input_batch = self.transform(frame).to(self.device)
            with torch.no_grad():
                prediction = self.midas(input_batch)
                # Resize back to original size
                prediction = torch.nn.functional.interpolate(
                    prediction.unsqueeze(1),
                    size=frame.shape[:2],
                    mode="bicubic",
                    align_corners=False,
                ).squeeze()
                depth_map = prediction.cpu().numpy()

            depth_min = depth_map.min()
            depth_max = depth_map.max()
            depth_vis = (255* (depth_map - depth_min) / (depth_max - depth_min)).astype(np.uint8)


        self.found_target=False

        if results[0].boxes and results[0].boxes.id is not None:
            boxes = results[0].boxes.xywh.cpu()
            track_ids = results[0].boxes.id.int().cpu().tolist()
            
            for box, track_id in zip(boxes, track_ids):
                x, y, w, h = box
                
                #check if there is a target and whether current id is target.
                if self.target_id == None:
                    self.target_id = track_id
                    
                if track_id == self.target_id:
                    self.found_target=True
                    
                    #Set centerpoint for target iD
                    self.target_cp.x = x
                    self.target_cp.y = y
                    self.target_cp.z = w * 70

                    #indicate target
                    cv2.putText(annotated_frame, f'Target ID: {int(track_id)}', (int(x - 0.5*w), int(y - 0.5*h - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    #Depth estimate
                    # if self.SIDE_use:
                    #     #Using SIDE depthmap
                    #     relative_distance = np.median(depth_map[int(y-0.5*h):int(y+0.5*h), int(x-0.5*w):int(x+0.5*w)])
                    #     self.target_cp.z = relative_distance
                    # else:
                    #     #Using box size
                    #     box_size=w*h
                    #     self.target_cp.z=box_size
                    
                    
                track = self.track_history[track_id]
                track.append((float(x), float(y)))
                if len(track) > 100:
                    track.pop(0)
                points = np.hstack(track).astype(np.int32).reshape((-1,1,2))
                cv2.polylines(annotated_frame, [points], isClosed=False, color=(230,230,230), thickness=2)
        
        #reset target if target lost
        if not self.found_target:
            self.target_id=None
            self.target_cp.x= 180.0
            self.target_cp.z= 10010.0

        cv2.putText(annotated_frame, f'Target at: (x={self.target_cp.x:.1f} y={self.target_cp.y:.1f}, d={self.target_cp.z:.1f})', (0,int(height-10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 2)

        #Publish target inputs to topic
        msg = Point()
        msg.x = float(self.target_cp.x)
        msg.y = 0.0
        
        # do or do not follow target based on distance
        msg.z = float(self.target_cp.z)
        #msg.z = 10001.0
        # # IDK yet how the z of the topic exactly works
        # msg.z = floatS(self.target_cp.z/600*1001)
        
        self.position_pub.publish(msg)
        # self.get_logger().debug(f'Published: ({msg.x:.1f}, area={msg.z:.1f})')
        
        frame_bgr = cv2.cvtColor(annotated_frame, cv2.COLOR_RGB2BGR)
        cv2.imshow('Input Stream', frame_bgr)
        cv2.imshow('depth map',depth_vis)
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
