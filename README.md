# Industrial-Mobile-Robot

This repository contains the development, control algorithms, Circuit Architecture and simulation setup for an Industrial Mobile Robot designed for both automotive and manual control operations.  
The robot is built using a Master-Slave Algorithm with I2C communication protocol for coordinated control between distributed microcontrollers.  
A fully parametric CAD model has been integrated into ROS and converted to URDF format, enabling teleoperation and visualization in simulation environments.

![](IMR_2D_SKETCH.png)



---

## 📘 Project Overview

The Industrial Mobile Robot operates under two control modes:  
1. Manual Control – teleoperation via ROS or joystick.  
2. Automotive Mode – semi-autonomous motion with sensor feedback.

The robot uses a distributed Master–Slave system, where:
- The Master Controller handles command logic, speed control, and I²C coordination.  
- The Slave Controllers manage the motors, sensors, and actuator feedback.  

This distributed structure improves response time, modularity, and system scalability.

---

## ⚙️ Key Features

- Master–Slave Algorithm: Multi-microcontroller coordination for motion and sensor tasks.  
- I²C Communication: Reliable serial data exchange between controllers.  
- Dual Mode Control: Switch between manual teleoperation and automotive (semi-autonomous) mode.  
- ROS–URDF Integration: The CAD model is converted to URDF for simulation in RViz and Gazebo.  
- Teleoperation Interface: Controlled through ROS nodes or Python scripts.  
- Sensor & Motor Fusion:Lidar, encoders, and motor drivers integrated for smooth operation.

---
🧮 System Components

| Component         | Description                                       |
| ----------------- | ------------------------------------------------- |
| Master Controller | Arduino Uno (I²C Master) |
| Slave Controllers | Arduino Uno      |
| Communication     | I²C (SDA, SCL) protocol                           |
| Motor Drivers     | bts7960                                           |
| Sensors           |  Encoders, optional LIDAR                         |
| Power Supply      | 12V lead cell battery                             |
| Control Modes     | Manual (Teleop) / Automotive                      |
| Software          | ROS, Python, Arduino                              |
| Simulation        | URDF + Gazebo                                     |



---

📧 Contact

For questions or collaborations, please contact:
📩 Suman Das Adhikary — [sumandasadhikary457@gmail.com]

🪪 License

MIT License © 2025
Use and modify with attribution.
