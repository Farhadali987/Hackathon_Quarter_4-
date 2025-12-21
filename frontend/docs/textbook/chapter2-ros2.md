---
title: Introduction to ROS 2
sidebar_position: 2
---

# Introduction to ROS 2

## What is ROS 2?

ROS 2 (Robot Operating System 2) is a flexible framework for developing robot applications. It is a rewrite of the original Robot Operating System (ROS) that addresses limitations in areas such as real-time support, embedded platforms, and commercial products.

## Key Features of ROS 2

### 1. Middleware Abstraction

ROS 2 uses a Data Distribution Service (DDS) as its middleware, which provides:

- **Real-time performance**: Support for real-time systems and time-sensitive applications
- **Distributed systems**: Better handling of distributed robot systems
- **Multiple implementations**: Options like Fast DDS, Cyclone DDS, RTI Connext DDS

### 2. Improved Architecture

- **Client libraries**: Support for multiple languages (C++, Python, etc.)
- **Package management**: Better dependency management and build systems
- **Security**: Built-in security features for production environments

### 3. Quality of Service (QoS)

ROS 2 provides Quality of Service settings that allow you to:

- Configure reliability (reliable vs. best-effort delivery)
- Set durability (volatile vs. transient-local)
- Control history (keep-all vs. keep-last)
- Define rate limits and resource usage

## Core Concepts

### Nodes

In ROS 2, a node is a process that performs computation. Nodes are organized in a graph structure and communicate with each other through topics, services, and actions.

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
```

### Topics and Messages

Topics are named buses over which nodes exchange messages. Messages are data structures that are passed between nodes.

### Services and Actions

- **Services**: Provide a request/reply communication pattern
- **Actions**: Offer goal-oriented communication with feedback and status updates

## Getting Started with ROS 2

### Installation

ROS 2 can be installed on Ubuntu, Windows, and macOS. The latest release is ROS 2 Humble Hawksbill (LTS).

### Basic Commands

- `ros2 run <package> <executable>`: Run a node
- `ros2 topic list`: List all topics
- `ros2 node list`: List all nodes
- `ros2 launch <package> <launch_file>`: Launch multiple nodes

## Practical Example: TurtleSim

TurtleSim is a simple simulator that allows you to control a turtle in a 2D space. It's an excellent tool for learning ROS 2 basics.

1. Start the turtlesim node: `ros2 run turtlesim turtlesim_node`
2. Control the turtle: `ros2 run turtlesim turtle_teleop_key`
3. Observe topics: `ros2 topic echo /turtle1/pose`

## ROS 2 in Physical AI and Humanoid Robotics

ROS 2 is particularly valuable in Physical AI and humanoid robotics because it:

- Provides standardized interfaces for sensors and actuators
- Offers extensive libraries for perception, planning, and control
- Supports simulation environments like Gazebo
- Enables modular development of complex robot behaviors

## Summary

ROS 2 is a powerful framework that provides the infrastructure needed to develop sophisticated robot applications. Its improved architecture addresses many limitations of the original ROS, making it suitable for both research and commercial applications in humanoid robotics and Physical AI.

## Next Steps

- Install ROS 2 and try the tutorials
- Experiment with the TurtleSim example
- Learn about ROS 2 packages and the build system (colcon)
- Explore simulation tools like Gazebo for robotics development