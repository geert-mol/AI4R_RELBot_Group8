# Step 1: Set RELBot ID in bashrc in docker:

`echo 'export ROS_DOMAIN_ID=25' >> ~/.bashrc`
`source ~/.bashrc`

# Step 2: SSH into RELBot with IP found on screen:

`ssh -X pi@192.168.0.100`

`ramforpresident`

# Step 3: Start G Streamer in SSH terminal

```commandline
gst-launch-1.0 -v \
v4l2src device=/dev/video2 ! \
image/jpeg,width=320,height=240,framerate=30/1 ! \
jpegdec ! videoconvert ! \
x264enc tune=zerolatency bitrate=800 speed-preset=ultrafast ! \
rtph264pay config-interval=1 pt=96 ! \
udpsink host=192.168.0.101 port=5000
```

# Step 4: Start DEMO script (in SSH)
```commandline
source ~/ai4r_ws/install/setup.bash
cd ~/ai4r_ws/
sudo ./build/demo/demo 
```

# Step 5: Start sequence controller (in SSH)
```commandline
source ~/ai4r_ws/install/setup.bash
ros2 launch sequence_controller sequence_controller.launch.py
```

# Step 6: Run video interface in docker (non-ssh)
`cd /ai4r_ws`

`colcon build --packages-select relbot_video_interface`

`source install/setup.bash`

`ros2 launch relbot_video_interface video_interface.launch.py`

# Test control the wheels:

`ros2 topic pub /object_position geometry_msgs/msg/Point "{x: 200.0, y: 0.0, z: 10001.0}" --once`


docker cp fea7deb3381d6fdb452f354d020e7eb36767b2f66e3d7a9ceeaa3ac8277c8dfc:/ai4r_ws/saved_frames /mnt/samsung/Coding/PycharmProjects/AI4R_RELBot/output



docker cp /mnt/samsung/Coding/PycharmProjects/AI4R_RELBot/models/no-retrain.pt fea7deb3381d6fdb452f354d020e7eb36767b2f66e3d7a9ceeaa3ac8277c8dfc:/ai4r_ws/