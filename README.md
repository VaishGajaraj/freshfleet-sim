# FreshFleet ROS Simulation Environment

A Docker-based ROS Noetic environment for FreshFleet simulation with UR10e robot arm and RealSense camera integration.

## Features

- **ROS Noetic** with Gazebo simulation
- **Universal Robots UR10e** with MoveIt integration
- **Intel RealSense D435i** camera support
- **Cross-platform compatibility** (Intel/Apple Silicon)
- **Pre-configured workspace** with all dependencies

## Prerequisites

- Docker Desktop installed and running
- For macOS: XQuartz for GUI support
- For Linux: X11 forwarding support

## Quick Start

### 1. Build the Docker Image

```bash
docker build -t freshfleet-ros .
```

### 2. Run the Container

#### macOS (with XQuartz)
```bash
# Start XQuartz and allow connections
xhost +localhost

# Run the container
docker run -it \
  --net=host \
  --env="DISPLAY=host.docker.internal:0" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  freshfleet-ros
```

#### Linux
```bash
docker run -it \
  --net=host \
  --env="DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  freshfleet-ros
```

### 3. Launch the Simulation

Inside the container:

```bash
# Terminal 1: Launch UR10e in Gazebo
roslaunch ur_gazebo ur10e_world.launch

# Terminal 2: Launch RealSense camera (simulated)
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud

# Terminal 3: Launch RViz for visualization
rosrun rviz rviz
```

## Development Workflow

### Adding Your Own ROS Packages

1. Create your ROS package in the host machine:
```bash
mkdir -p src/freshfleet_detection
cd src/freshfleet_detection
catkin_create_pkg freshfleet_detection rospy sensor_msgs geometry_msgs
```

2. Mount your source code into the container:
```bash
docker run -it \
  --net=host \
  --env="DISPLAY=host.docker.internal:0" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$(pwd)/src:/root/catkin_ws/src" \
  freshfleet-ros
```

3. Build your workspace:
```bash
cd /root/catkin_ws
catkin_make
source devel/setup.bash
```

### Using Real Hardware

To use a physical RealSense camera, mount the device:

```bash
docker run -it \
  --net=host \
  --env="DISPLAY=host.docker.internal:0" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --device=/dev/video0 \
  --device=/dev/video1 \
  --device=/dev/video2 \
  freshfleet-ros
```

## Useful Commands

### ROS Topics
```bash
# List all topics
rostopic list

# View camera data
rostopic echo /camera/color/image_raw

# View robot joint states
rostopic echo /joint_states

# View point cloud data
rostopic echo /camera/depth/points
```

### MoveIt Integration
```bash
# Launch MoveIt with UR10e
roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch

# Launch MoveIt RViz plugin
roslaunch ur10_moveit_config moveit_rviz.launch
```

### Gazebo Commands
```bash
# Pause simulation
rosservice call /gazebo/pause_physics

# Resume simulation
rosservice call /gazebo/unpause_physics

# Reset simulation
rosservice call /gazebo/reset_simulation
```

## Troubleshooting

### GUI Issues on macOS
If you encounter GUI issues on macOS:

1. Install XQuartz: `brew install --cask xquartz`
2. Start XQuartz and go to Preferences → Security → Allow connections from network clients
3. Restart XQuartz
4. Use the macOS run command above

### Permission Issues
If you encounter permission issues with the RealSense camera:

```bash
# Add your user to the video group (Linux)
sudo usermod -a -G video $USER

# Or run with privileged mode (not recommended for production)
docker run --privileged ...
```

### Build Issues
If the Docker build fails:

1. Check your internet connection
2. Try building with more memory allocated to Docker
3. Clear Docker cache: `docker system prune -a`

## Project Structure

```
freshfleet-sim/
├── Dockerfile              # Main Docker configuration
├── ros_entrypoint.sh       # ROS environment setup
├── README.md              # This file
├── docker-compose.yml     # Optional: Multi-container setup
└── src/                   # Your ROS packages (mounted)
    └── freshfleet_detection/
        ├── package.xml
        ├── setup.py
        └── scripts/
```

## Next Steps

1. **Implement detection algorithms** in your ROS package
2. **Add URDF models** for your specific use case
3. **Create launch files** for your complete system
4. **Add unit tests** for your ROS nodes
5. **Set up CI/CD** for automated testing

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test in the Docker environment
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details. 