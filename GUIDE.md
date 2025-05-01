SSH into the RELBot with:

`ssh -X pi@192.168.0.105`

# Start G Streamer

Run this in your normal terminal when connected to normal internet

`sudo apt install net-tools`

Then connect to RaM-lab and run:

`ifconfig`

Save the IP somewhere and use it in an SSH terminal with:

```commandline
gst-launch-1.0 -v \
v4l2src device=/dev/video2 ! \
image/jpeg,width=640,height=480,framerate=30/1 ! \
jpegdec ! videoconvert ! \
x264enc tune=zerolatency bitrate=800 speed-preset=ultrafast ! \
rtph264pay config-interval=1 pt=96 ! \
udpsink host=192.168.0.118 port=5000
```

Then in docker (non-ssh):

```commandline
gst-launch-1.0 -v udpsrc port=5000 caps="application/x-rtp,media=video,encoding-name=H264,payload=96" ! rtph264depay ! avdec_h264 ! autovideosink
```

# Run video interface
In docker (non-ssh)
`cd /ai4r_ws`
`colcon build --packages-select relbot_video_interface`
`source install/setup.bash`
`ros2 run `