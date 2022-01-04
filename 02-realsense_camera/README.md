# Isaac ROS - Realsense camera

# Build

```
bash script/build_docker.sh 02-realsense_camera
```



# Run docker

```
docker run --runtime nvidia --network host rbonghi/isaac-ros-tutorial:realsense-camera
```

Option explained:
* --runtime nvidia will use the NVIDIA container runtime while running the l4t-base container
* --network host To read all topics outside the container need to share the same network to the host
* -v is the mounting directory