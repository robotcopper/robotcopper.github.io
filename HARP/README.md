---
title: HARP project
time: 2024-09-23
---

<div style="display: flex; justify-content: center;">
    <img src="/config/assets/images/HARP/Harp_logo_medallion.webp" style="background: transparent;" width="15%" >
</div>

<center>
  <h1 style="margin-top: 3px; padding-bottom: 30px; line-height: 0.8;">
    <span style="font-family: 'IgnisEtGlaciesSharp'!important; font-size: 60px; font-weight: bold;">H.A.R.P.</span><br>
    <span style="font-size: 24px;">(Holonomic Autonomous Robotic Platform)</span>
  </h1>
</center>

<br>

<span style="font-family: 'IgnisEtGlaciesSharp'!important;font-weight: bold;">HARP</span> (Holonomic Autonomous Robotic Platform) is my primary ongoing project:

<div style="flex: 1; min-width: 300px; background-color: #A3C6C4; padding: 10px; border-radius: 5px; color: #333; font-family: Arial, sans-serif; line-height: 1.6;">
  <hr style="border: none; border-top: transparent; margin: 9px 0; background: transparent;">
  <p style="color: #000; text-indent: 20px;">
    Launched in July 2022, <span style="font-family: 'IgnisEtGlaciesSharp'!important;">HARP</span> was inspired by my first experience participating in the French Robotics Cup with the <a href="https://eirbot.github.io/" rel="noreferrer" target="_blank"
      style="color:#000; text-decoration: none;" 
      onmouseover="this.style.color='#3498DB'; this.style.textDecoration='underline';" 
      onmouseout="this.style.color='#000'; this.style.textDecoration='none';">
      EIRBOT
    </a> robotics association at my engineering school. Since its inception, I have been working on the project independently, financing it entirely with personal funds.<br> Given the significant investment—both financially and in terms of time—I've focused on utilizing affordable technology and leveraging existing solutions, whenever possible, rather than reinventing the wheel. This led me to adopt <span style="font-weight: bold;">ROS</span>(Robot Operating System) from the very beginning.<br><br>
    My ultimate goal is to <span style="font-weight: bold;">demystify robotics</span> by demonstrating that cutting-edge robotics can be accessible to everyone.
  </p>
</div>

<br>
HARP is designed for participation in [**Eurobot**](https://www.eurobot.org/), integrating **Mechatronics** (electronics, mechanics, software, and control systems), **Programming**, and **Machine Learning**. 

<div style="flex: 1; min-width: 300px; background-color: #BADDDD; padding: 10px; border-radius: 5px; color: #333; font-family: Arial, sans-serif; line-height: 1.6;">
  <hr style="border: none; border-top: transparent; margin: 9px 0; background: transparent;">
  <p style="color: #000; text-indent: 20px;">
    The first version of my robot <a href="https://robotcopper.github.io/HARP/HARP1/" rel="noreferrer" target="_blank"
    style="color:#000; text-decoration: none; font-family: 'IgnisEtGlaciesSharp'!important; font-weight: bold;" 
    onmouseover="this.style.color='#3498DB'; this.style.textDecoration='underline';" 
    onmouseout="this.style.color='#000'; this.style.textDecoration='none';">
    HARP1</a>, was built using ROS1 Melodic and primarily leveraged simple communication between a Raspberry Pi and an Arduino Nano through <span style="font-weight: bold;">rosserial</span>, enabling basic topic-based communication. Despite the robot's holonomic structure (three omnidirectional wheels), only differential drive functionality could be implemented in this version.
  </p>
</div>

<div style="flex: 1; min-width: 300px; background-color: #C5E8E8; padding: 10px; border-radius: 5px; color: #333; font-family: Arial, sans-serif; line-height: 1.6; margin-top: 5px;">
  <hr style="border: none; border-top: transparent; margin: 9px 0; background: transparent;">
  <p style="color: #000; text-indent: 20px;">
    The second iteration <a href="https://robotcopper.github.io/HARP/HARP2/" rel="noreferrer" target="_blank"
  style="color:#000; text-decoration: none; font-family: 'IgnisEtGlaciesSharp'!important; font-weight: bold;" 
  onmouseover="this.style.color='#3498DB'; this.style.textDecoration='underline';" 
  onmouseout="this.style.color='#000'; this.style.textDecoration='none';">
  HARP2</a>, which is currently under development, has been upgraded to <span style="font-weight: bold;">ROS2 Humble</span>. This version incorporates the <span style="font-weight: bold;">NAV2 stack</span> for trajectory generation and dynamic obstacle avoidance, <span style="font-weight: bold;">ros2_control</span> for inverse kinematics, and <span style="font-weight: bold;">Kalman filter sensor fusion</span> via <span style="font-weight: bold;">robot_localization</span> for accurate odometry. Future iterations will include <span style="font-weight: bold;">MoveIt</span> for actuator manipulation.
  </p>
</div>

<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;On the mechanical side, I design and fabricate parts using a combination of **laser cutting** and **3D printing**. All hardware is custom-built, from schematic design and PCB routing to soldering. For <span style="font-family: 'IgnisEtGlaciesSharp'!important;font-weight: bold;">HARP1</span>, I even handled the PCB manufacturing process myself, from etching to assembly. The project covers a broad spectrum of disciplines, ranging from low-level transistor-based circuits to high-level control systems and machine learning algorithms.

I’m deeply invested in understanding the full scope of the technologies involved, dedicating time to studying the underlying code, libraries, and relevant research papers. This project is a perfect outlet for my passion for **systems engineering**, where every component must be thoughtfully integrated into the overall system of the robot.

<div style="display: flex; justify-content: center;">
    <video style="width: 100%; max-width: 850px; display: block;" controls>
        <source src="/config/assets/images/HARP/video_project_V3_compress.mp4" type="video/mp4">
        Your browser does not support the video tag.
    </video>
</div>

<br>
To learn more about the <span style="font-family: 'IgnisEtGlaciesSharp'!important;font-weight: bold;">HARP1</span> and <span style="font-family: 'IgnisEtGlaciesSharp'!important;font-weight: bold;">HARP2</span> robots, feel free to explore the following article.
