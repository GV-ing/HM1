# 🤖 HM1 Project: Armando Robotic Arm Control and Simulation (ROS 2 Humble)

This repository contains the complete solution for **Homework 1** of the Robotics Lab class, focused on building ROS packages to simulate a **4-degrees-of-freedom robotic manipulator arm (Armando)** within the **Gazebo environment**.

The solution addresses all four main problems (modeling, sensing, controllers, and autonomous C++ control node).A detailed point-to-point solution report must also be provided.

## 📂 Repository Structure

HM1
├README
│
├PKG/
│ │
│ ├ armando_description/  
│ ├ armando_gazebo/      
│ └ armando_controller/  
│ 
└Documentation/
 ├README
 │
 ├─ 1  
 │	├1.a
 │	├1.b
 │	└1.c   
 ├─ 2  
 │	├2.a
 │	├2.b
 │	└2.c        
 ├─ 3  
 │	├3.a
 │	├3.b
 │	└3.c   
 └─ 4  
    ├4.a
    ├4.b
    ├4.c    
    └4.d
     	
     	
     	
# 🦾 HM1 Setup & Usage Guide


To make the code work correctly, it's necessary to download only the following packages: armando_description, armando_gazebo and armando_controller. It is essential **not to install** (or delete after installation) the **Documentation** folder.

To open the HM1:

---

## 🚀 0. Install All Necessary Libraries

```bash
sudo apt update
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-urdf-launch
colcon build
source install/setup.bash
```

---

## 🤖 1. armando_description

The robot spawns into **Rviz** with a modified collision box.

### 🧩 Launch Command

```bash
ros2 launch armando_description armando_display.launch.py
```

---

## 🌍 2–3. armando_gazebo

This package spawns the robot in **Gazebo** and **Rviz**, with configured camera and position control.

### 🚀 Launch Simulation

```bash
ros2 launch armando_gazebo armando_world.launch.py
```

### 🎮 Send a Control Command

You can manually publish a topic to control the robot’s joints.

#### Example Command

```bash
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.0, 0.5, -1.0, -0.5]}"
```

---

## 🧠 4. armando_controller

Creation of a **C++ node** that allows us to visualize the joint states of the robot.

### ▶️ Run the Controller Node

```bash
ros2 run armando_controller arm_controller_node
```

---

## ⚙️ 4.d Control Type Selection

Implementation of the ability to choose between two different control types — `joint_trajectory_controller` and `position_controller` — directly from the terminal,  
with a default option of `position_controller`.

### 🧩 Example 1 — Using `joint_trajectory_controller`

```bash
ros2 launch armando_gazebo armando_world.launch.py controller_type:=joint_trajectory_controller
```
```bash
ros2 run armando_controller arm_controller_node --ros-args -p publisher_type:=trajectory
```

### ⚙️ Example 2 — Using Default `position_controller`

```bash
ros2 launch armando_gazebo armando_world.launch.py 
```
```bash
ros2 run armando_controller arm_controller_node
```

---




