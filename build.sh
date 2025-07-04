#!/bin/bash

# FreshFleet ROS Docker Build Script
# This script builds and runs the FreshFleet ROS simulation environment

set -e

echo "🚀 Building FreshFleet ROS Docker environment..."

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo "❌ Docker is not running. Please start Docker Desktop first."
    exit 1
fi

# Build the Docker image
echo "📦 Building Docker image..."
docker build -t freshfleet-ros .

if [ $? -eq 0 ]; then
    echo "✅ Docker image built successfully!"
else
    echo "❌ Docker build failed!"
    exit 1
fi

# Check OS for appropriate run command
if [[ "$OSTYPE" == "darwin"* ]]; then
    echo "🍎 Detected macOS - using XQuartz setup"
    
    # Check if XQuartz is installed
    if ! command -v xquartz &> /dev/null; then
        echo "⚠️  XQuartz not found. Installing..."
        brew install --cask xquartz
    fi
    
    echo "🔧 Setting up X11 forwarding for macOS..."
    xhost +localhost
    
    echo "🐳 Starting FreshFleet ROS container..."
    docker run -it \
        --net=host \
        --env="DISPLAY=host.docker.internal:0" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="$(pwd)/src:/root/catkin_ws/src:rw" \
        freshfleet-ros

elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
    echo "🐧 Detected Linux - using X11 forwarding"
    
    echo "🐳 Starting FreshFleet ROS container..."
    docker run -it \
        --net=host \
        --env="DISPLAY" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="$(pwd)/src:/root/catkin_ws/src:rw" \
        freshfleet-ros

else
    echo "❌ Unsupported operating system: $OSTYPE"
    echo "Please run the Docker command manually:"
    echo "docker run -it --net=host --env=\"DISPLAY=host.docker.internal:0\" --volume=\"/tmp/.X11-unix:/tmp/.X11-unix:rw\" --volume=\"$(pwd)/src:/root/catkin_ws/src:rw\" freshfleet-ros"
    exit 1
fi

echo "👋 Container stopped. Thanks for using FreshFleet ROS!" 