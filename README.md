# RoboticsLab - LAMPO Robot Simulation

This repository contains ROS 2 packages for the LAMPO robot simulation using Gazebo and Nav2 navigation stack.

## Prerequisites

- ROS 2 Kilted

## Building the Workspace

To compile the ROS workspace with colcon:

```bash
source /opt/ros/kilted/setup.bash 
cd ros_ws
colcon build --symlink-install
source install/setup.bash
```

The `--symlink-install` flag allows you to modify Python scripts and configuration files without rebuilding.

## Running the Simulation

### Launch Simulation and Visualization

To start the Gazebo simulation and RViz visualizer:

```bash
cd ros_ws
source install/setup.bash
ros2 launch lampo_description lampo_gz_mm.launch.py
```

This launcher starts:
- Gazebo simulation with the LAMPO robot model
- RViz for visualization
- Robot state publisher
- Required bridges between ROS 2 and Gazebo

### Launch Navigation Stack

To start the navigation system with Nav2(in another terminal):

```bash
cd ros_ws
source install/setup.bash
ros2 launch lampo_description lampo_nav_omni.launch.py
```

This launcher initializes:
- Nav2 navigation stack
- AMCL localization
- Path planning and control
- Navigation parameters for omnidirectional movement

## Windows Users - Devcontainer Setup with VcXsrv

Windows users can run this project using VSCode devcontainers with graphical support via VcXsrv.

### Prerequisites


1. Install [VSCode](https://code.visualstudio.com/) 
2. Install [Docker Desktop for Windows](https://www.docker.com/products/docker-desktop)
3. Install the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) in VSCode, search it in the left bar extension tab
4. Install [VcXsrv](https://sourceforge.net/projects/vcxsrv/)

### Setup Instructions

1. **Configure VcXsrv:**
   - Launch XLaunch from the Start menu
   - Select "Multiple windows" and set display number to 0
   - Select "Start no client"
   - **Important:** Check "Disable access control"
   - Save the configuration for future use

2. **Allow VcXsrv through Windows Firewall:**
   - Open Windows Defender Firewall
   - Click "Allow an app through firewall"
   - Find VcXsrv and enable both Private and Public networks

3. **Get your Windows host IP:**
   - Open PowerShell and run: `ipconfig`
   - Note your IPv4 address (typically 192.168.x.x or similar)

4. **Configure the devcontainer:**
   - Open the project in VSCode
   - The `.devcontainer` configuration should already be set up
   - Ensure the `DISPLAY` environment variable is correctly set in `.devcontainer/devcontainer.json`

5. **Open in devcontainer:**
   - Press `F1` or `Ctrl+Shift+P` to open the command palette
   - Type and select: "Dev Containers: Reopen in Container"
   - Wait for the container to build and start

6. **Test the GUI:**
   - Once inside the container, ensure VcXsrv is running on Windows
   - Try launching the simulation as described above
   - The Gazebo and RViz windows should appear on your Windows desktop

### Troubleshooting

- **No display appearing:** Verify VcXsrv is running and "Disable access control" is checked
- **Connection refused:** Check Windows Firewall settings for VcXsrv
- **Display offset issues:** Ensure the DISPLAY variable in devcontainer matches your VcXsrv display number (typically :0)

