# 🚀 Autonomous Navigation Pipeline – FAST-LIO, LIO-SAM, Nav2 & Isaac Sim

This repository contains a complete **mapless autonomous navigation pipeline** built in **NVIDIA Isaac Sim** using **ROS2 Nav2**, along with experimental evaluation of **FAST-LIO** (LiDAR–IMU odometry) and **LIO-SAM** (LiDAR SLAM).  
The robot navigates to any target **without a pre-built map**, relying entirely on **2D LiDAR-based local costmaps** and real-time obstacle avoidance.

This project demonstrates end-to-end integration between:
- Isaac Sim → ROS2 sensor bridge  
- FAST-LIO / LIO-SAM → Odometry / SLAM  
- Nav2 → Mapless navigation with local planners  

---

## 🟦 Project Overview

This repository includes:

- **Mapless Navigation** using ROS2 Nav2  
- **Real-time obstacle avoidance** using simulated 2D Rotary LiDAR  
- **3D LiDAR odometry** using FAST-LIO  
- **3D LiDAR SLAM** using LIO-SAM (loop closure + pose graph)  
- **Isaac Sim sensor bridging** (IMU, 2D LiDAR, 3D LiDAR)  
- Nav2 **local planner comparison** → RPP, DWB, TEB  
- Automated pipeline for:  
  - Launching FAST-LIO  
  - Launching LIO-SAM  
  - Running Nav2 with correct odometry source  

---

## 🟩 Features

- 🗺️ **Mapless Navigation** – No global map required  
- 🛰️ **LiDAR–IMU Odometry (FAST-LIO)**  
- 🔁 **LiDAR SLAM with loop closure (LIO-SAM)**  
- 🚧 **Real-time obstacle detection** using 2D LiDAR  
- 🔧 **Nav2 Local Planners:** RPP, DWB, TEB  
- ⛓️ **TF2 Transform Tree** (base_link, odom, laser, imu)  
- 🌐 **Isaac Sim → ROS2** integration  
- 🧩 Modular launch files for each pipeline  

---

## 🔧 Tools & Technologies

**ROS2, Nav2, Isaac Sim, Ouster 3D LiDAR (OS0/OS1), NVIDIA Rotary 2D LiDAR, FAST-LIO, LIO-SAM, TF2, RViz2**

---

## 🛠 System Architecture

Isaac Sim Sensors → ROS2 Bridge → LiDAR/IMU Topics → Nav2 (Local Planner)
| ↑
└→ FAST-LIO / LIO-SAM (Odometry / SLAM)

yaml
Copy code

- **2D LiDAR** → Nav2 costmap  
- **3D LiDAR** → FAST-LIO / LIO-SAM  
- **IMU** → FAST-LIO / LIO-SAM  
- **Odometry** → Nav2 Motion Planning  

---

# ▶️ **How to Run the Project**

You can run the navigation pipeline in two modes:

---

# 🟦 **1️⃣ FAST-LIO + Nav2 (Mapless Navigation)**

### **Step 1 — Start Isaac Sim**
Run your Isaac Sim scene with:
- /lidar_scan  
- /imu  
- /odom  

---

### **Step 2 — Run the Ring Converter Node**  
Isaac Sim 3D LiDAR does NOT publish the **ring field**, but FAST-LIO requires it.

ros2 run pointcloud_ring_converter ring_converter_node.py

Step 3 — Launch FAST-LIO Mapping

This runs odometry and publishes /cloud_registered + /odom.
ros2 launch fast_lio_nav2 mapping.launch.py

Step 4 — Launch Nav2 (FAST-LIO Odometry Mode)

ros2 launch fast_lio_nav2 nav2_fastlio.launch.py
This uses FAST-LIO’s /odom as the robot’s localization source.

Step 5 — Send Navigation Goal
Open RViz2:
rviz2
Use the Nav2 Goal tool → click anywhere on the mapless environment → robot moves using:
FAST-LIO odometry
2D LiDAR costmap
Local planners (RPP / DWB / TEB)

🟩 2️⃣ LIO-SAM + Nav2 (Mapless Navigation)
Step 1 — Start Isaac Sim
Same as the previous mode.

Step 2 — Run the IMU Upsampler
LIO-SAM requires a high-frequency IMU.

ros2 run pointcloud_ring_converter imu_upsampling.py

Step 3 — Launch LIO-SAM
ros2 launch fast_lio_nav2 run.launch.py

This starts:
Factor graph
Loop closure
Keyframe optimization
Publishing /lio_sam/mapping/pose

Step 4 — Launch Nav2 (LIO-SAM Mode)
ros2 launch fast_lio_nav2 nav2_liosam.launch.py
Nav2 now uses LIO-SAM’s optimized pose for navigation.

Step 5 — Navigate
Use RViz2 → Send Nav2 goal → Robot moves using:
LIO-SAM SLAM pose
2D LiDAR costmap
Local planner (RPP/DWB/TEB)

🟨 3️⃣ Pure Mapless Nav2 (Without SLAM)
If you want to run only Nav2 with 2D LiDAR:

ros2 launch fast_lio_nav2 nav2_fastlio.launch.py
Robot navigates using:
2D LiDAR → costmap
Isaac Sim → odometry (if provided)
Local planner only

🧠 Key Learnings
FAST-LIO vs LIO-SAM → Odometry vs SLAM

IMU–LiDAR fusion pipeline

Why mapless navigation uses only local planners

How LiDAR costmaps work in Nav2

TF2 setup for Isaac Sim → ROS2 navigation

Launching and integrating SLAM + Navigation stacks

📫 Contact
If you're working on ROS2, SLAM, LiDAR processing, or Isaac Sim, feel free to reach out!
Happy to collaborate on robotics and autonomous systems.
I'm looking for job opportunities. Please refer me.

LinkedIn:- www.linkedin.com/in/shareefbaba
Gmail:- shareefbaba1404@gmail.com








