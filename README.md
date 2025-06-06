# AI4R_RELBot

**AI for Autonomous Robot** Course  
University of Twente — Assignment 1

This repository contains the source code and setup instructions for **Assignment 1** of the AI for Autonomous Robot course at the University of Twente. You will extend the RAM department’s hardware module (RELBot) with AI-driven capabilities, including object detection, tracking, and SLAM, all implemented using modern deep-learning techniques.

All development should occur within the provided Docker container. Preserve your code snippets and submit them as ROS 2 packages for evaluation.

---
# ROS2 Package: `relbot_video_interface`

This skeleton package captures a GStreamer video stream and publishes object position messages to the RELBot. It exposes the video capture loop and the `/object_position` topic to control the RELBot.

---

## Prerequisites

- **Ubuntu** host (tested on 20.04+)
- **RELBot** hardware module

### Preparing the Host PC with Docker container

Before you begin, clone this repository (which includes the `Dockerfile`, the `assignment1_setup.sh` script, and example ROS2 source code) and use the provided script to build and launch your Docker environment:

```bash
# 1. Clone the repo and enter it
git clone https://github.com/UAV-Centre-ITC/AI4R_RELBot.git
cd AI4R_RELBot

# 2. Make the helper script executable
chmod +x assignment1_setup.sh

# 3. Run the script to build (first run) or attach (subsequent runs):
./assignment1_setup.sh  # use in every shell where you want a ROS2 Docker session
```

This script will:

- Build the Docker image (named by the script) on first invocation.
- Start or attach to an existing container on future runs.
- Mount your host workspace into the container at `/ros2_ws/src` by default.

The default mapping is controlled by these variables at the top of `assignment1_setup.sh`:

```bash
HOST_FOLDER="${1:-$(pwd)/ai4r_ws/src}"    # host path to mount (defaults to ./ai4r_ws/src)
CONTAINER_FOLDER="/ros2_ws/src"           # container mount point
```

Adjust those or add extra `-v <host_dir>:<container_dir>` flags if you need additional mounts. (You have to remove and start the container again to see the effect)

Inside the container, you’ll find your workspace at `/ros2_ws/src`.  All packages you create there will persist on the host.


---

## Step 1: Connect to the RELBot

1. **WLAN (default)**  
   - The RELBot auto-connects to SSID **RAM-lab**.  
   - Ensure your host PC is on **RAM-lab**.  

2. **Ethernet (alternative)**  
   - Connect a standard Ethernet cable between your Ubuntu host and the RELBot.  
   - In **Settings > Network > Wired**, set IPv4 Method to **"Shared to other computers"** (this shares your host’s connection via DHCP).  
   - Check your host’s interface and assigned IP with:
     ```
     ifconfig  # look under eth0, enp*, or similar
     ```
   - The RELBot’s Ethernet IP appears on its LCD; alternatively, view connected devices via:
     ```
     arp -n
     ```

3. **Determine IPs**  
   - **RELBot IP**: read from the LCD.  
   - **Host IP**: run `ifconfig` and note the address under your active interface (`wlan0`, `eth0`, etc.).  

4. **SSH access**  
   ```
   ssh pi@<RELBOT_IP>
   # For GUI apps, enable X11 forwarding:
   ssh -X pi@<RELBOT_IP>
   ```

> **Tip:** Open three terminals on the RELBot. VS Code’s Remote SSH extension can simplify this.

---

## Step 2: Stream External Webcam Video

The RELBot’s external webcam is typically available at `/dev/video2`. If you encounter errors or no video, list all video devices and choose the one labeled “Webcam.”

1. **List video devices**  
   ```
   v4l2-ctl --list-devices
   ```
   _Example output (Webcam section only):_
   ```
   Webcam: Webcam (usb-0000:01:00.0-1.4):
       /dev/video2
       /dev/video3
       /dev/media1
   ```
   Here, use `/dev/video2` for MJPEG streaming.

2. **Determine your host IP**  
   ```
   ifconfig  # look under wlan0 or eth0
   ```

3. **Start the GStreamer pipeline** _(replace `<HOST_IP>` with your host IP)_  
   ```
   gst-launch-1.0 -v \
   v4l2src device=/dev/video2 ! \
   image/jpeg,width=640,height=480,framerate=30/1 ! \
   jpegdec ! videoconvert ! \
   x264enc tune=zerolatency bitrate=800 speed-preset=ultrafast ! \
   rtph264pay config-interval=1 pt=96 ! \
   udpsink host=<HOST_IP> port=5000
   ```
   - **v4l2src**: captures MJPEG frames.  
   - **jpegdec + videoconvert**: decode to raw video.  
   - **x264enc**: encode to H.264 with low latency.  
   - **rtph264pay**: packetize for RTP.  
   - **udpsink**: stream to your host.

---

## Step 3: Launch Controller Nodes on the RELBot

Prebuilt ROS 2 packages for the FPGA and Raspberry Pi controllers are provided with your RELBot hardware. Report hardware issues for a replacement; software bugs will be patched across all robots.

**Terminal 1:**
```
source ~/ai4r_ws/install/setup.bash
cd ~/ai4r_ws/
sudo ./build/demo/demo   # low-level motor and FPGA interface
```

**Terminal 2:**
```
source ~/ai4r_ws/install/setup.bash
ros2 launch sequence_controller sequence_controller.launch.py   # high-level state machine
```

---

## Step 4: Configure Your Docker Container

Set up your ROS 2 workspace to communicate with the RELBot via a unique domain ID.

1. **Preview the video stream**  
   ```
   gst-launch-1.0 -v \
   udpsrc port=5000 caps="application/x-rtp,media=video,encoding-name=H264,payload=96" ! \
   rtph264depay ! avdec_h264 ! autovideosink
   ```

2. **Set your ROS domain**  
   ```
   export ROS_DOMAIN_ID=<RELBot_ID>   # e.g., 8 for RELBot08
   ```
   Add this line to `~/.bashrc` to make it persistent.

---

###  Build and Test relbot_video_interface Package

The `relbot_video_interface` package is already included in this repository under `/ai4r_ws/src/relbot_video_interface` and is mounted into the Docker container at `/ros2_ws/src/relbot_video_interface`.

Once your Docker container is running and prerequisites (ROS 2, GStreamer, etc.) are satisfied, build and launch the test node to verify the video stream:

```bash
# From inside the container, at the workspace root:
cd /ai4r_ws

# 1) Build only the provided package:
colcon build --packages-select relbot_video_interface

# 2) Source workspace:
source install/setup.bash

# 3) Launch the video interface node:
ros2 launch relbot_video_interface video_interface.launch.py
```

You should see the live camera feed streaming and placeholder `/object_position` messages being published. If no errors occur, the setup is correct and you can proceed to add or modify detection logic.


Happy coding—and good luck with your assignment!  

---
## ROS2 Cheatsheet

Below are common ROS 2 commands and tools for development and debugging within the Docker container:

```bash
# Build and source workspace
colcon build
source /opt/ros/jazzy/setup.bash
source /ai4r_ws/install/setup.bash

# List available packages
ros2 pkg list

# Run a node directly
ros2 run relbot_video_interface video_interface

# Launch using a launch file
ros2 launch relbot_video_interface video_interface.launch.py

# Inspect running nodes and topics
ros2 node list
ros2 topic list
ros2 topic echo /object_position

# Manage node parameters
ros2 param list /video_interface
ros2 param get /video_interface gst_pipeline
ros2 param set /video_interface gst_pipeline "<new_pipeline_string>"

# Publish a test message once
ros2 topic pub /object_position geometry_msgs/msg/Point "{x: 100.0, y: 0.0, z: 0.0}" --once

# Debug with RQT tools
rqt                  # launch RQT plugin GUI
rqt_graph            # visualize topic-node graph

# View TF frames (if using tf2)
ros2 run tf2_tools view_frames.py
# opens frames.pdf showing TF tree

# Record and playback topics
ros2 bag record /object_position  <other_topics>    # start recording
ros2 bag play <bagfile>               # play back recorded data and process it offline
```
---
## VS Code Development Environment

For seamless development, you can use VS Code’s Remote extensions to work directly inside your Docker container and SSH into the RELBot:

1. **Install Extensions 
   - **Remote Development**

2. **Connect to the Docker Container**  
   - Press <kbd>F1</kbd> → **Dev-Containers: Attach to Running Container…**  
   - Select your `relbot_ai4r_assignment1` container.  
   - VS Code will reopen with the container filesystem as your workspace.

3. **SSH into the RELBot**  
   - In the **Remote Explorer** sidebar, under **Remotes(SSH/Tunnels)**, click **Configure button** to add a host entry for `pi@<RELBOT_IP>`.  
   - Press <kbd>F1</kbd> → **Remote-SSH: Connect to Host…** → select your RELBot entry.  
   - VS Code opens a new window connected to the robot, letting you inspect logs or edit files over SSH.

4. **Workflow Tips**  
   - Use split windows: one VS Code window attached to Docker (for code build & debug), another attached to RELBot (for robot-side logs & tweaks).  
   - Ensure `~/.ssh/config` has your RELBot entry for quick access:  
     ```text
     Host  <RELBot_IP>
         HostName <RELBot_IP>
         ForwardX11 yes
         User pi
     ```
