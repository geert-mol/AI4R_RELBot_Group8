# Step 1: Set RELBot ID in bashrc in docker:

`echo 'export ROS_DOMAIN_ID=<RELBot_ID>' >> ~/.bashrc`
`source ~/.bashrc`

# Step 2: SSH into RELBot with IP found on screen:

`ssh -X pi@10.1.1.209`

# Step 3: Start G Streamer in SSH terminal

```commandline
gst-launch-1.0 -v \
v4l2src device=/dev/video2 ! \
image/jpeg,width=320,height=240,framerate=30/1 ! \
jpegdec ! videoconvert ! \
x264enc tune=zerolatency bitrate=800 speed-preset=ultrafast ! \
rtph264pay config-interval=1 pt=96 ! \
udpsink host=10.1.1.244 port=5000
```

# Step 4: Start other G Streamer (non-SSH)

```commandline
gst-launch-1.0 -v udpsrc port=5000 caps="application/x-rtp,media=video,encoding-name=H264,payload=96" ! rtph264depay ! avdec_h264 ! autovideosink
```

# Step 5: Start DEMO script
```commandline
source ~/ai4r_ws/install/setup.bash
cd ~/ai4r_ws/
sudo ./build/demo/demo 
```

# Step 6: Start sequence controller
```commandline
source ~/ai4r_ws/install/setup.bash
ros2 launch sequence_controller sequence_controller.launch.py
```

# Step 7: Run video interface in docker (non-ssh)
`cd /ai4r_ws`
`colcon build --packages-select relbot_video_interface`
`source install/setup.bash`
`ros2 launch relbot_video_interface video_interface.launch.py`

# Test control the wheels:

`ros2 topic pub /object_position geometry_msgs/msg/Point "{x: 200.0, y: 0.0, z: 10001.0}" --once`