# ROS 2 Multi-Sensor Perception System

This project implements a **real-time perception system** using **multiple USB cameras** and a **real lidar sensor**, built with the ROS 2 framework.

It supports:
- Multiple real camera feeds
- Manual camera selection via mouse click
- Real-time human detection (HOG-based)
- Real sensor data from lidar (planned or ongoing)
- Modular node-based architecture

---

## 🚀 Features

- 🧠 **Human Detection:** Detects and highlights people using OpenCV's HOG detector
- 🎥 **Multi-Camera Support:** 3 simultaneous USB cameras
- 🖱️ **Interactive Selection:** Select active camera via OpenCV window click
- 📡 **ROS 2 Integration:** Uses `rclpy`, `sensor_msgs`, `cv_bridge`, and image topics
- 🔍 **Real Sensor Input:** Works with real hardware, not simulation

---

## 🧩 System Requirements

- Ubuntu 20.04 or later
- ROS 2 Foxy, Humble or Iron
- Python 3.8+
- USB cameras (at least 2 recommended)
- Optional: Real lidar sensor (Velodyne, RPLidar etc.)

---

## 📦 Dependencies

Install required packages:

```bash
sudo apt install ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-image-transport
pip install opencv-python
```

---

## 🛠️ How to Run

### 1. Clone the repo and build:

```bash
cd ~/ros2_ws/src
git clone https://github.com/fatihboken/ros2-multisensor-perception.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 2. Run the camera node:

```bash
ros2 run camera_node camera_node
```

### 3. Click on a camera window to activate human detection for that feed.

> Detected people will be shown with green bounding boxes.

---

## 🧠 Future Plans

- Add real lidar point cloud processing
- Sensor fusion (camera + lidar)
- Multi-object tracking
- Map integration with SLAM

---

## 👤 Author

Fatih Böken  
[GitHub](https://github.com/fatihboken)

---

## 📜 License

MIT License
