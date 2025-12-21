---
title: Cloud Lab 1 - Setting Up Your Robotics Development Environment
---

# Cloud Lab 1: Setting Up Your Robotics Development Environment

## Objective

This lab will guide you through setting up a cloud-based development environment for robotics development using the NVIDIA Isaac platform. You will learn to configure a cloud computing environment, set up ROS 2, and create a simulation environment for testing humanoid robotics algorithms.

## Prerequisites

Before starting this lab, you should have:
- A cloud computing account (AWS, Azure, or GCP)
- Basic familiarity with Linux command line
- Understanding of robotics concepts (covered in previous chapters)
- Access to NVIDIA Isaac development resources

## Learning Objectives

After completing this lab, you will be able to:
1. Set up a cloud computing instance with GPU support
2. Install and configure ROS 2 in a cloud environment
3. Deploy NVIDIA Isaac tools and libraries
4. Configure simulation environments for robotics testing
5. Establish secure remote access to your development environment

## Cloud Instance Setup

### Step 1: Selecting the Right Instance Type (10 minutes)

1. **Navigate to your cloud provider's console**:
   - AWS: EC2 Dashboard
   - Azure: Virtual Machines
   - GCP: Compute Engine

2. **Select an instance with GPU support**:
   - AWS: Choose a `g4dn.xlarge` or larger instance
   - Azure: Choose an `NCv3` or `NDv2` series instance
   - GCP: Choose an `n1-standard` with Tesla T4 GPUs

3. **Configure instance specifications**:
   - CPU: At least 4 vCPUs
   - Memory: 16GB or more
   - Storage: 100GB SSD recommended
   - GPU: 1x NVIDIA T4 or equivalent

4. **Set up security groups/firewall**:
   - Allow SSH (port 22)
   - Allow HTTP (port 80) if needed
   - Allow custom ports for robotics applications

### Step 2: Initial Instance Configuration (15 minutes)

Connect to your instance and run these commands:

```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Install essential tools
sudo apt install -y build-essential cmake pkg-config
sudo apt install -y git curl wget vim htop tmux
sudo apt install -y python3-dev python3-pip python3-venv

# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER

# Install Docker Compose
sudo curl -L "https://github.com/docker/compose/releases/download/v2.20.0/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose

# Reboot to ensure all configurations take effect
sudo reboot
```

### Step 3: NVIDIA Driver and CUDA Setup (20 minutes)

After rebooting, install the NVIDIA drivers:

```bash
# Add NVIDIA package repositories
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.0-1_all.deb
sudo dpkg -i cuda-keyring_1.0-1_all.deb
sudo apt-get update

# Install CUDA toolkit
sudo apt-get install -y cuda-toolkit-12-0

# Install NVIDIA Container Toolkit for Docker
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

# Verify installation
nvidia-smi
```

## ROS 2 Installation

### Step 4: Installing ROS 2 Humble Hawksbill (25 minutes)

```bash
# Add ROS 2 GPG key and repository
sudo apt update && sudo apt install -y curl gnupg
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y python3-colcon-common-extensions
sudo apt install -y python3-rosdep python3-vcstool

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2 environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 5: Setting Up Your Workspace (15 minutes)

Create and configure your ROS 2 workspace:

```bash
# Create workspace directory
mkdir -p ~/robotics_ws/src
cd ~/robotics_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build the workspace
colcon build --symlink-install

# Source the workspace
echo "source ~/robotics_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## NVIDIA Isaac Setup

### Step 6: Installing NVIDIA Isaac Tools (20 minutes)

```bash
# Install Isaac ROS common dependencies
sudo apt install -y ros-humble-ament-cmake-catch2
sudo apt install -y ros-humble-image-transport-plugins
sudo apt install -y ros-humble-vision-opencv

# Clone Isaac ROS repository
cd ~/robotics_ws/src
git clone -b humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone -b humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone -b humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git
git clone -b humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_point_cloud_processor.git

# Install additional dependencies
cd ~/robotics_ws
rosdep install --from-paths src --ignore-src -r -y

# Build Isaac ROS packages
colcon build --symlink-install --packages-select \
  isaac_ros_common \
  isaac_ros_visual_slam \
  isaac_ros_image_pipeline \
  isaac_ros_point_cloud_processor
```

### Step 7: Installing Isaac Sim (30 minutes)

For simulation capabilities, install Isaac Sim in a Docker container:

```bash
# Pull Isaac Sim Docker image
docker pull nvcr.io/nvidia/isaac-sim:4.0.0

# Create a directory for Isaac Sim data
mkdir -p ~/isaac_sim_data

# Create a startup script
cat << 'EOF' > ~/start_isaac_sim.sh
#!/bin/bash
xhost +local:docker

docker run --gpus all -it --rm \
  --network=host \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="~/isaac_sim_data:/isaac_sim_data" \
  --privileged \
  --expose=30001 \
  --publish=30001:30001 \
  --expose=30002 \
  --publish=30002:30002 \
  --expose=5000 \
  --publish=5000:5000 \
  --expose=5001 \
  --publish=5001:5001 \
  --expose=50051 \
  --publish=50051:50051 \
  --expose=55555 \
  --publish=55555:55555 \
  nvcr.io/nvidia/isaac-sim:4.0.0
EOF

chmod +x ~/start_isaac_sim.sh
```

## Gazebo Setup

### Step 8: Installing Gazebo Garden (15 minutes)

```bash
# Add Gazebo repository
sudo curl -sSL http://get.gazebosim.org | sh

# Install Gazebo Garden
sudo apt install gz-garden

# Install ROS 2 Gazebo bridge
sudo apt install ros-humble-gazebo-ros-pkgs

# Test installation
gz sim -v
```

## Development Environment Configuration

### Step 9: Setting Up VS Code Remote Development (20 minutes)

1. **Install VS Code locally** on your personal computer
2. **Install the Remote-SSH extension** in VS Code
3. **Configure SSH access** to your cloud instance:
   ```bash
   # Generate SSH key on your local machine
   ssh-keygen -t rsa -b 4096
   
   # Copy public key to your cloud instance
   ssh-copy-id username@your-instance-ip
   ```

4. **Connect VS Code to your cloud instance** using the Remote-SSH extension

### Step 10: Installing Useful Development Tools (10 minutes)

```bash
# Install additional development tools
pip3 install jupyter notebook matplotlib pandas numpy scipy

# Install ROS 2 tools
sudo apt install -y ros-dev-tools
sudo apt install -y ros-humble-rqt ros-humble-rviz2

# Install version control tools
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

## Testing Your Setup

### Step 11: Running a Test Simulation (25 minutes)

Create a simple test to verify your setup:

```bash
# Create a test directory
mkdir -p ~/robotics_ws/src/test_simulation
cd ~/robotics_ws/src/test_simulation

# Create a simple launch file
cat << 'EOF' > test_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker',
            output='screen'
        ),
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='listener',
            output='screen'
        )
    ])
EOF

# Build the workspace
cd ~/robotics_ws
colcon build --packages-select test_simulation

# Source the workspace
source install/setup.bash

# Run the test
ros2 launch test_simulation test_launch.py
```

## Cloud-Specific Optimizations

### Step 12: Setting Up Auto-Shutdown and Cost Management (10 minutes)

Create a script to automatically shut down the instance when not in use:

```bash
# Create a shutdown script
cat << 'EOF' > ~/auto_shutdown.sh
#!/bin/bash
# Auto-shutdown script to save costs
# Usage: ./auto_shutdown.sh [minutes]

MINUTES=${1:-60}  # Default to 60 minutes if no argument provided

echo "Instance will shut down in $MINUTES minutes. Press Ctrl+C to cancel."
sleep $((MINUTES * 60))
sudo shutdown -h now
EOF

chmod +x ~/auto_shutdown.sh

# Create a reminder script
cat << 'EOF' > ~/cost_reminder.sh
#!/bin/bash
# Cost management reminder
echo "Remember to shut down your instance when not in use:"
echo "sudo shutdown -h now"
echo ""
echo "Or use the auto-shutdown script:"
echo "~/auto_shutdown.sh [minutes]"
EOF

chmod +x ~/cost_reminder.sh
echo "~/cost_reminder.sh" >> ~/.bashrc
```

## Troubleshooting Common Issues

### Issue 1: GPU Not Detected
**Symptoms**: `nvidia-smi` command fails or doesn't show GPU
**Solutions**:
1. Check that your instance type actually has a GPU
2. Verify NVIDIA drivers are properly installed
3. Reboot the instance to ensure drivers load properly

### Issue 2: ROS 2 Commands Not Found
**Symptoms**: Commands like `ros2` or `colcon` not recognized
**Solutions**:
1. Verify ROS 2 is properly sourced in your bashrc
2. Run `source /opt/ros/humble/setup.bash` manually
3. Check that installation completed without errors

### Issue 3: Isaac Sim Display Issues
**Symptoms**: Isaac Sim doesn't start with GUI or shows errors
**Solutions**:
1. Ensure X11 forwarding is properly configured
2. Check that your local machine has a display available
3. Try running in headless mode if display is not needed

## Best Practices for Cloud Robotics Development

### Resource Management
- **Monitor usage**: Regularly check CPU, GPU, and memory usage
- **Right-size instances**: Scale up/down based on current needs
- **Use spot instances**: For non-critical workloads to reduce costs

### Security
- **Keep systems updated**: Regularly update OS and packages
- **Secure access**: Use SSH keys and restrict access to necessary IPs
- **Network isolation**: Use VPCs and security groups appropriately

### Collaboration
- **Version control**: Use Git for all code and configurations
- **Documentation**: Keep detailed records of setup and changes
- **Sharing**: Use containerization to share reproducible environments

## Lab Assessment

### Verification Steps
1. Verify ROS 2 is installed: `ros2 --version`
2. Check GPU status: `nvidia-smi`
3. Test Isaac tools: `dpkg -l | grep isaac`
4. Verify Gazebo installation: `gz sim -v`

### Expected Outcomes
- Successful installation of ROS 2 Humble
- Working NVIDIA GPU with CUDA support
- Access to Isaac ROS tools and libraries
- Functional simulation environment (Gazebo or Isaac Sim)
- Properly configured development environment

## Extension Activities

1. **Deploy a sample robot simulation**: Load a humanoid robot model in Gazebo
2. **Implement a simple controller**: Create a basic movement controller
3. **Integrate with VLA models**: Connect your environment to vision-language-action models
4. **Set up CI/CD**: Create automated testing pipelines for your robotics code

## Summary

This lab provided a comprehensive setup for cloud-based robotics development. You now have:
- A GPU-enabled cloud instance configured for robotics
- ROS 2 Humble installed and configured
- NVIDIA Isaac tools and libraries available
- Simulation environments ready for testing
- A properly configured development environment

## Next Steps

- Explore the Isaac Sim tutorials
- Try running sample robotics projects
- Set up your first humanoid robot simulation
- Begin implementing the projects from previous textbook chapters in your cloud environment

Remember to manage your cloud costs by shutting down instances when not in use!