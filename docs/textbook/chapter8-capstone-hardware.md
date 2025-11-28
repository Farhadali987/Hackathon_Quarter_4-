---
sidebar_position: 9
---

# Chapter 8: Capstone Project & Hardware Requirements

This course culminates in a comprehensive Capstone Project, where all the learned concepts are integrated to bring an autonomous humanoid robot to life in a simulated environment. This chapter outlines the project and details the essential hardware required to successfully undertake this technically demanding course.

## Capstone Project: The Autonomous Humanoid

The Capstone Project challenges students to develop a simulated humanoid robot capable of understanding and executing complex tasks through natural language interaction. It's a synthesis of all modules, demonstrating a complete AI-driven physical system.

**Project Scenario:** A simulated robot receives a voice command, plans a path, navigates obstacles, identifies an object using computer vision, and manipulates it.

**Key Components to Integrate:**

1.  **Voice-to-Text:** Using an ASR system (e.g., OpenAI Whisper, as discussed in Chapter 5) to convert a spoken command into a text instruction.
2.  **Cognitive Planning (LLM Integration):** Employing a Large Language Model (as discussed in Chapter 5 and 7) to translate the high-level natural language instruction (e.g., "Bring me the blue cup from the table") into a sequence of specific, executable ROS 2 actions.
3.  **Perception (Isaac ROS):** Utilizing GPU-accelerated computer vision algorithms (from Chapter 4) to:
    *   **Locate Objects:** Identify and localize the target object (e.g., "blue cup") within the simulated environment.
    *   **Environment Mapping:** Continuously update the robot's understanding of its surroundings.
4.  **Navigation (Nav2):** Implementing the ROS 2 Nav2 stack (from Chapter 4) to:
    *   **Global Path Planning:** Generate a collision-free path from the robot's current location to the target object.
    *   **Local Path Execution:** Control the robot's bipedal locomotion (from Chapter 6) to follow the planned path while avoiding dynamic obstacles.
5.  **Manipulation and Grasping:** Developing strategies (from Chapter 6) for the humanoid's end-effectors to:
    *   **Approach the Object:** Position the robot's hand correctly relative to the target object.
    *   **Grasp the Object:** Execute a stable grasp.
    *   **Transport and Place:** Move the object to the designated location.
6.  **ROS 2 Orchestration:** All these components will communicate and be coordinated through the ROS 2 middleware (from Chapter 2), ensuring seamless data flow and command execution.
7.  **Simulation Environment:** The entire project will be developed and tested within NVIDIA Isaac Sim or Gazebo (from Chapter 3), leveraging their photorealistic rendering and accurate physics.

This capstone project provides a hands-on experience in building a truly intelligent and embodied robotic system, reflecting the core themes of physical AI.

## Hardware Requirements

This course is technically demanding, sitting at the intersection of three heavy computational loads: Physics Simulation (Isaac Sim/Gazebo), Visual Perception (SLAM/Computer Vision), and Generative AI (LLMs/VLA). To ensure a productive learning experience, specific hardware is essential.

### 1. The "Digital Twin" Workstation (Required per Student)

This is the most critical component for running high-fidelity simulations. NVIDIA Isaac Sim is an Omniverse application that requires "RTX" (Ray Tracing) capabilities. Standard laptops (MacBooks or non-RTX Windows machines) will not work effectively.

*   **GPU (The Bottleneck):** NVIDIA RTX 4070 Ti (12GB VRAM) or higher.
    *   **Why:** High VRAM is needed to load USD (Universal Scene Description) assets for complex robot models and environments, plus run VLA models simultaneously.
    *   **Ideal:** RTX 3090 or 4090 (24GB VRAM) allows for smoother "Sim-to-Real" training and larger models.
*   **CPU:** Intel Core i7 (13th Gen+) or AMD Ryzen 9.
    *   **Why:** Physics calculations (Rigid Body Dynamics) in Gazebo/Isaac are CPU-intensive.
*   **RAM:** 64 GB DDR5 (32 GB is the absolute minimum, but will likely crash during complex scene rendering).
*   **OS:** Ubuntu 22.04 LTS.
    *   **Note:** While Isaac Sim runs on Windows, ROS 2 (Humble/Iron) is native to Linux. Dual-booting or dedicated Linux machines are mandatory for a friction-free experience.

### 2. The "Physical AI" Edge Kit (Recommended for Hands-on Experience)

Since a full humanoid robot is expensive, students learn "Physical AI" by setting up a robust compute platform before deploying it to a robot. This kit covers Modules 3 (Isaac ROS) and 4 (VLA) in a practical context.

*   **The Brain:** NVIDIA Jetson Orin Nano (8GB) or Orin NX (16GB).
    *   **Role:** This is the industry standard for embodied AI. Students will deploy their ROS 2 nodes here to understand resource constraints versus their powerful workstations.
*   **The Eyes (Vision):** Intel RealSense D435i or D455.
    *   **Role:** Provides RGB (Color) and Depth (Distance) data. Essential for the VSLAM and Perception modules.
*   **The Inner Ear (Balance):** Generic USB IMU (BNO055) (Often built into the RealSense D435i or Jetson boards, but a separate module helps teach IMU calibration).
*   **Voice Interface:** A simple USB Microphone/Speaker array (e.g., ReSpeaker) for the "Voice-to-Action" Whisper integration.

### 3. The Robot Lab (Optional, for Physical Deployment)

For the "Physical" part of the course, three tiers of options depend on budget. The software principles transfer effectively, even with proxy robots.

*   **Option A: The "Proxy" Approach (Recommended for Budget)**
    *   **Robot:** Unitree Go2 Edu (~$1,800 - $3,000). Highly durable, excellent ROS 2 support, affordable. (Note: Not bipedal).
*   **Option B: The "Miniature Humanoid" Approach**
    *   **Robot:** Unitree G1 (~$16k) or Robotis OP3 (~$12k). Budget Alternative: Hiwonder TonyPi Pro (~$600, but limited AI processing).
*   **Option C: The "Premium" Lab (Sim-to-Real specific)**
    *   **Robot:** Unitree G1 Humanoid. One of the few commercially available humanoids with an open SDK for student ROS 2 controllers.

## Summary of Architecture

To successfully teach this course, a robust lab infrastructure is key:

| Component       | Hardware                          | Function                                                                 |
| :-------------- | :-------------------------------- | :----------------------------------------------------------------------- |
| **Sim Rig**     | PC with RTX 4080 + Ubuntu 22.04   | Runs Isaac Sim, Gazebo, Unity, trains LLM/VLA models.                    |
| **Edge Brain**  | Jetson Orin Nano                  | Runs the "Inference" stack. Students deploy their code here.             |
| **Sensors**     | RealSense Camera + Lidar          | Connected to the Jetson to feed real-world data to the AI.               |
| **Actuator**    | Unitree Go2 or G1 (Shared)        | Receives motor commands from the Jetson.                                 |

**Cloud-Native Lab ("Ether" Lab - High OpEx Option):**
If RTX-enabled workstations are unavailable, the course can rely on cloud-based instances (e.g., AWS RoboMaker, NVIDIA's Omniverse Cloud), though this introduces latency and cost.
*   **Cloud Workstations:** AWS g5.2xlarge (A10G GPU, 24GB VRAM) or g6e.xlarge.
*   **Local "Bridge" Hardware:** Still requires Edge AI Kits (Jetson) for physical deployment.

The "Economy Jetson Student Kit" offers a cost-effective way to learn ROS 2, basic computer vision, and Sim-to-Real control, comprising an NVIDIA Jetson Orin Nano Super Dev Kit, Intel RealSense D435i, ReSpeaker USB Mic Array, and necessary accessories for approximately $700.

This chapter ensures students are prepared both conceptually and practically for the demands of Physical AI and humanoid robotics.