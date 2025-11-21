#!/bin/bash
# Source ROS 2 workspace automatically in the devcontainer

if ! grep -Fxq "source /opt/ros/kilted/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/kilted/setup.bash" >> ~/.bashrc
fi

echo "ROS 2 environment ready."