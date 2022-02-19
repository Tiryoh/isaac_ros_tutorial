# Isaac ROS - Realsense camera

# Build

```
bash scripts/build_docker.sh 02-realsense_camera
```



# Run docker

```
docker run --runtime nvidia --network host --privileged rbonghi/isaac-ros-tutorial:realsense-camera
```

Option explained:
* --runtime nvidia will use the NVIDIA container runtime while running the l4t-base container
* --network host To read all topics outside the container need to share the same network to the host
* -v is the mounting directory
*  --privileged Allow access to host

## Advanced options

To launch with other RealSense like D415 and D435, add an `realsense_type` argument.

```
docker run --runtime nvidia --network host --privileged rbonghi/isaac-ros-tutorial:realsense-camera \
ros2 launch isaac_ros_realsense isaac_ros_realsense.launch.py realsense_type:=d415
```
