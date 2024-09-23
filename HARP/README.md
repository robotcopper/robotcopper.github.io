---
title: HARP project
time: 2024-09-23
---

<center>
 <h1 style="font-size: 36px; font-weight: bold;"> HARP (Holonomic Autonomous Robotic Platform) </h1>
</center>

<br>

**HARP** (Holonomic Autonomous Robotic Platform) is my primary ongoing project:

Launched in July 2022, HARP was inspired by my first experience participating in the French Robotics Cup with the [EIRBOT](https://eirbot.github.io/) robotics association at my engineering school. Since its inception, I have been working on the project independently, financing it entirely with personal funds. Given the significant investment—both financially and in terms of time—I've focused on utilizing affordable technology and leveraging existing solutions, whenever possible, rather than reinventing the wheel. This led me to adopt **ROS** (Robot Operating System) from the very beginning.

My ultimate goal is to **demystify robotics** by demonstrating that cutting-edge robotics can be accessible to everyone.

HARP is designed for participation in [**Eurobot**](https://www.eurobot.org/), integrating **Mechatronics** (electronics, mechanics, software, and control systems), **Programming**, and **Machine Learning**. 

The first version of my robot, [**HARP1**](https://robotcopper.github.io/HARP/HARP1/), was built using ROS1 Melodic and primarily leveraged simple communication between a Raspberry Pi and an Arduino Nano through **rosserial**, enabling basic topic-based communication. Despite the robot's holonomic structure (three omnidirectional wheels), only differential drive functionality could be implemented in this version.

The second iteration, [**HARP2**](https://robotcopper.github.io/HARP/HARP2/), which is currently under development, has been upgraded to **ROS2 Humble**. This version incorporates the **NAV2 stack** for trajectory generation and dynamic obstacle avoidance, **ros2_control** for inverse kinematics, and **Kalman filter sensor fusion** via **robot_localization** for accurate odometry. Future iterations will include **MoveIt** for actuator manipulation.

On the mechanical side, I design and fabricate parts using a combination of **laser cutting** and **3D printing**. All hardware is custom-built, from schematic design and PCB routing to soldering. For **HARP1**, I even handled the PCB manufacturing process myself, from etching to assembly. The project covers a broad spectrum of disciplines, ranging from low-level transistor-based circuits to high-level control systems and machine learning algorithms.

I’m deeply invested in understanding the full scope of the technologies involved, dedicating time to studying the underlying code, libraries, and relevant research papers. This project is a perfect outlet for my passion for **systems engineering**, where every component must be thoughtfully integrated into the overall system of the robot.

To learn more about the **HARP1 and HARP2 project**, feel free to explore the following article.
