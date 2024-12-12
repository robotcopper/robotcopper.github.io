---
title: HARP2
time: 2024-12-11
---

<div style="display: flex; flex-wrap: wrap; align-items: stretch; background-color: #f0d95c; padding: 10px; border-radius: 10px; font-family: Arial, sans-serif; color: #000; max-width: 92%; box-sizing: border-box; margin: 0 auto; position: relative; overflow: hidden;">
    <!-- Horn effect -->
    <div style="content: ''; position: absolute; top: 0; left: 0; width: 15%; max-width: 90px; aspect-ratio: 1; background: linear-gradient(-45deg, transparent 50%, #d7b845 50%); clip-path: polygon(0 0, 100% 0, 0% 100%);"></div>
    <!-- Text section -->
    <div style="flex: 1 1 60%; margin-right: 20px; min-width: 280px;">
        <center><div style="margin-top: 0; font-size: 36px; font-weight: bold; line-height: 1.2; border-bottom: 1.5px solid #fff; padding-bottom: 5px;">2023-now&nbsp; – &nbsp;<span style="font-family: 'IgnisEtGlaciesSharp'!important; font-size: 42px;">HARP2</span></div></center>
        <center><div style="font-size: 21px; font-weight: bold; margin: 1em 0;">Under Development</div></center>
        <p style="padding-left: 25px;">
        <strong>Principle:</strong> <span style="font-family: 'IgnisEtGlaciesSharp'!important;">HARP2</span> is designed for seamless navigation and full autonomy, independently making decisions to adapt dynamically to its environment. Its ultimate goal is to autonomously manipulate game elements, demonstrating its versatile and multidisciplinary capabilities.        
        </p>
        <div style="font-size: 21px; font-weight: bold; margin: 1em 0; padding-left: 25px;">Specifications:</div>
        <ul style="padding-left: 100px;">
            <li>3 Nema 17 stepper motors (59Ncm) for propulsion</li>
            <li>Li-Po 14.8V 8400mAh battery</li>
            <li>2 Raspberry pi Pico microcontrollers and 1 Raspberry Pi 4B+ connected via USB</li>
            <li>Chassis made of 1 cm aluminum profiles and laser-cut PMMA</li>
            <li>3D-printed mechanical parts</li>
            <li>2 2D LiDAR for obstacle detection, navigation and odometry</li>
            <li>1 IMU and 1 Optical Tracking Odometry Sensor</li>
        </ul>
    </div>
    <!-- Video section -->
    <div style="flex: 1 1 35%; min-width: 280px; display: flex; align-items: stretch;">
        <div style="width: 100%; position: relative;">
            <video style="position: absolute; top: 0; left: 0; width: 100%; height: 100%; background-color: black;" controls>
                <source src="/config/assets/images/HARP/HARP2/video_project.mp4" type="video/mp4">
                Your browser does not support the video tag.
            </video>
        </div>
    </div>
</div>

<br>
<br>

<style>
@media (max-width: 768px) {
    .container_harp img {
      display: none !important;
    }
}
</style>

<div style="display: flex; align-items: center; gap: 20px;overflow: hidden; max-width: 90%;" class="container_harp">
  <img src="/config/assets/images/HARP/HARP2/HARP2_emblem2.png" style="max-width: 250px; max-height: 250px; background-color: transparent; display: block; margin-left: 13%;">
  <div style="display: flex; flex-wrap: wrap; justify-content: center; gap: 12px;">
    <a href="#overview-diagram" class="button_harp default">
      OVERVIEW DIAGRAM
    </a>
    <a href="#mechanics" class="button_harp default">
      MECHANICS
    </a>
    <a href="#electronics" class="button_harp default">
      ELECTRONICS
    </a>
    <a href="#micro-controller" class="button_harp default">
      uC & uROS
    </a>
    <a href="#ros" class="button_harp default">
      ROS (Robot Operating System)
    </a>
    <a href="#actuator" class="button_harp default">
      ACTUATOR
    </a>
  </div>
</div>

<br>
<br>

<!-- Example of the sections -->
<h2 id="overview-diagram" style="line-height: 1.2; border-bottom: 1.5px solid #5f3db1; margin-left: 10%;">Overview diagram</h2>
  <div style="display: flex; justify-content: center;">
      <img src="/config/assets/images/HARP/HARP2/HARP2_poster.png" style="background: transparent; border-radius: 20px; width: 60%" >
  </div>
  <p align="center" style="color:#a6a6a6;">Workflow and components of the robot HARP2 (Holonomic Autonomous Robotic Platform 2)</p>

<br>

<h2 id="mechanics" style="line-height: 1.2; border-bottom: 1.5px solid #5f3db1; margin-right: 10%; text-align: -moz-right;">Mechanics</h2>

  <div style="display: flex; align-items: center; gap: 20px;overflow: hidden;" class="container_harp">
    <div style="display: flex; flex-wrap: wrap; justify-content: center; gap: 12px;">
      <p style="color: #d0d0d0; line-height: 35px; text-indent: 20px;">This robot was designed using Onshape, a CAD software. Its structure consists of 3mm laser-cut PMMA layers and assembled using 1cm aluminum profiles for durability and rigidity. Additional components are crafted from 3D-printed PLA, ensuring lightweight and customizable elements. The overall dimensions strictly adhere to the regulations of the French Robotics Cup.</p>
    </div>
    <img src="/config/assets/images/HARP/HARP2/Harp2_meca.jpg" style="max-width: 350px; max-height: 250px; background-color: transparent; display: block; border-radius: 7px;">
  </div>

<h2 id="electronics" style="line-height: 1.2; border-bottom: 1.5px solid #5f3db1; margin-left: 10%;">Electronics</h2>
  <div style="font-style: italic; margin-left: 5%; font-size: 1.2em; font-weight: bold;">Power Board</div>
  <div style="display: flex; align-items: center; gap: 20px; overflow: hidden; max-width: 85%; margin-left: 7%;" class="container_harp">
    <div style="display: grid; grid-template-columns: repeat(2, 1fr); gap: 0px; max-width: 18%; max-height: 18%; border-radius: 3px; overflow: hidden;">
      <img src="/config/assets/images/HARP/HARP2/PowerBoard_PCB.png" style="width: 100%; height: 100%; background-color: transparent;">
      <img src="/config/assets/images/HARP/HARP2/PowerBoard_Schematic.png" style="width: 100%; height: 100%; background-color: transparent;">
      <img src="/config/assets/images/HARP/HARP2/PowerBoard_3D.png" style="width: 100%; height: 100%; background-color: transparent;">
      <img src="/config/assets/images/HARP/HARP2/PowerBoard_Photo.jpg" style="width: 100%; height: 100%; background-color: transparent;">
    </div>
    <div style="display: flex; flex-wrap: wrap; justify-content: center; gap: 12px;">
      <p style="color: #d0d0d0; line-height: 35px; text-indent: 20px;">The power board plays a crucial role in distributing power from the battery to various modules while adapting the electrical parameters as needed. It was designed using KiCad, an open-source software for printed circuit board (PCB) design. Following the validation, the PCB was manufactured. In addition to onboard protections, such as fuses, short-circuit protection, and thermal safeguards, a low-voltage buzzer alarm was added to the battery to enhance its protection.</p>
    </div>
  </div>

  <div style="text-align: -moz-right; font-style: italic; margin-right: 5%; font-size: 1.2em; font-weight: bold;">Emergency Stop Button Board</div>
  <div style="display: flex; align-items: center; gap: 20px; overflow: hidden; max-width: 85%; margin-left: 7%;" class="container_harp">
    <div style="display: flex; flex-wrap: wrap; justify-content: center; gap: 12px;">
      <p style="color: #d0d0d0; line-height: 35px; text-indent: 20px;">The Emergency Stop Button Board ensures the safe operation of the system by integrating a relay controlled by two push buttons. One button serves as the activation switch to power on the system, while the other functions as an emergency stop button, immediately cutting off power when pressed. This board is a critical safety feature, providing a reliable and straightforward way to quickly halt the system in case of unexpected issues. Its design prioritizes simplicity and robustness, ensuring it meets high safety standards.</p>
    </div>
    <div style="display: grid; grid-template-columns: repeat(2, 1fr); gap: 0px; max-width: 18%; max-height: 18%; border-radius: 3px; overflow: hidden;">
      <img src="/config/assets/images/HARP/HARP2/BAU_PCB.png" style="width: 100%; height: 100%; background-color: transparent;">
      <img src="/config/assets/images/HARP/HARP2/BAU_Schematic.png" style="width: 100%; height: 100%; background-color: transparent;">
      <img src="/config/assets/images/HARP/HARP2/BAU_3D.png" style="width: 100%; height: 100%; background-color: transparent;">
      <img src="/config/assets/images/HARP/HARP2/BAU_Photo.jpg" style="width: 100%; height: 100%; background-color: transparent;">
    </div>
  </div>

  <div style="font-style: italic; margin-left: 5%; font-size: 1.2em; font-weight: bold;">Mobile Base Electronic Board</div>
  <div style="display: flex; align-items: center; gap: 20px; overflow: hidden; max-width: 85%; margin-left: 7%;" class="container_harp">
    <div style="display: grid; grid-template-columns: repeat(2, 1fr); gap: 0px; max-width: 18%; max-height: 18%; border-radius: 3px; overflow: hidden;">
      <img src="/config/assets/images/HARP/HARP2/StepperDriver_board_PCB.png" style="width: 100%; height: 100%; background-color: transparent;">
      <img src="/config/assets/images/HARP/HARP2/StepperDriver_board_Schematic.png" style="width: 100%; height: 100%; background-color: transparent;">
      <img src="/config/assets/images/HARP/HARP2/StepperDriver_board_3D.png" style="width: 100%; height: 100%; background-color: transparent;">
      <img src="/config/assets/images/HARP/HARP2/StepperDriver_board_Photo.jpg" style="width: 100%; height: 100%; background-color: transparent;">
    </div>
    <div style="display: flex; flex-wrap: wrap; justify-content: center; gap: 12px;">
      <p style="color: #d0d0d0; line-height: 35px; text-indent: 20px; text-indent: 20px;">The Mobile Base Electronic Board integrates connections to MKS-SERVO42C V1.1 drivers for controlling the three 59 Ncm NEMA 17 stepper motors. Additionally, it embeds a Raspberry Pi Pico, the limit switches located at the base of the robot, and the Optical Tracking Odometry Sensor. <br> 
      Therefore, this board receives motor commands and transmits limit switch statuses and odometry data to the Raspberry Pi. To enhance feedback, LEDs were added to indicate the activation of each limit switch for better monitoring.</p>
    </div>
  </div>

  <div style="text-align: -moz-right; font-style: italic; margin-right: 5%; font-size: 1.2em; font-weight: bold;">IMU Board</div>
  <div style="display: flex; align-items: center; gap: 20px; overflow: hidden; max-width: 85%; margin-left: 7%;" class="container_harp">
    <div style="display: flex; flex-wrap: wrap; justify-content: center; gap: 12px;margin-left: 40%;">
      <div style="background-color:#f0ad4e; color:#ffffff; padding:10px; text-align:center; font-size:20px; font-weight:bold; border-radius:5px; box-shadow: 0px 2px 5px rgba(0, 0, 0, 0.2);">
        <img src="https://img.icons8.com/ios-filled/50/000000/under-construction.png" style="vertical-align: middle; margin-right: 10px; background-color: transparent;">
        Work in progress
        <img src="https://img.icons8.com/ios-filled/50/000000/under-construction.png" style="vertical-align: middle; margin-right: 10px; background-color: transparent;">
      </div>
      <!-- <p style="color: #d0d0d0; line-height: 35px; text-indent: 20px;">Work in progress</p> -->
    </div>
    <!-- <div style="display: grid; grid-template-columns: repeat(2, 1fr); gap: 0px; max-width: 18%; max-height: 18%; border-radius: 3px; overflow: hidden;">
      <img src="/config/assets/images/HARP/HARP1/StepperDriver_board_PCB.png" style="width: 100%; height: 100%; background-color: transparent;">
      <img src="/config/assets/images/HARP/HARP1/StepperDriver_board_Schematic.png" style="width: 100%; height: 100%; background-color: transparent;">
      <img src="/config/assets/images/HARP/HARP1/StepperDriver_board_3D.png" style="width: 100%; height: 100%; background-color: transparent;">
      <img src="/config/assets/images/HARP/HARP1/StepperDriver_board_Photo.jpg" style="width: 100%; height: 100%; background-color: transparent;">
    </div> -->
  </div>

<br>

<h2 id="micro-controller" style="line-height: 1.2; border-bottom: 1.5px solid #5f3db1; margin-left: 10%;">Micro Controller and micro-ROS</h2>

  <div style="display: flex; align-items: center; gap: 20px;overflow: hidden;" class="container_harp">
    <div style="display: flex; flex-wrap: wrap; justify-content: center; gap: 12px;">
      <p style="color: #d0d0d0; line-height: 35px; text-indent: 20px;">Communication between the Raspberry Pi Pico units and the robot’s central processing unit, the Raspberry Pi 4B+, is established via USB using the micro-ROS protocol. In this setup, the Raspberry Pi Picos act as clients, while the Raspberry Pi functions as the server. This configuration enables seamless transmission of commands and sensor data between the Raspberry Pi and multiple Raspberry Pi Pico interfaces. <br><br>
      The implementation of micro-ROS ensures efficient communication and data synchronization across devices. This architecture allows the Raspberry Pi to handle complex, high-level computations and decision-making, while the Raspberry Pi Pico units serve as low-level interfaces, each managing specific hardware components and interactions.</p>
    </div>
    <img src="/config/assets/images/HARP/HARP2/picoxuros.png" style="max-width: 300px; max-height: 250px; background-color: transparent; display: block; border-radius: 7px;">
  </div>

<br>

<h2 id="ros" style="line-height: 1.2; border-bottom: 1.5px solid #5f3db1; margin-right: 10%; text-align: -moz-right;">ROS (Robot Operating System)</h2>
  <div style="display: flex; align-items: center; gap: 20px;overflow: hidden;" class="container_harp">
    <img src="/config/assets/images/HARP/HARP1/ros_logo.png" style="max-width: 350px; max-height: 250px; background-color: transparent; display: block; border-radius: 12px;">
    <div style="display: flex; flex-wrap: wrap; justify-content: center; gap: 12px;">
      <p style="color: #d0d0d0; line-height: 35px; text-indent: 20px;"><span style="font-family: 'IgnisEtGlaciesSharp'!important;">HARP2</span> currently operates under ROS 2 Humble, integrating sensor drivers into its ROS architecture while leveraging the Navigation2 and ros2_control stacks. This setup enables the use of advanced, state-of-the-art algorithms for path planning, navigation, obstacle avoidance, and decision-making.<br>
      Additionally, the integration of the MoveIt2 stack is planned for actuator control, further enhancing the robot's capabilities. The system is also simulated in Gazebo, providing a platform to test and refine strategies before deployment.<br><br>
      This approach fully exploits the flexibility and modularity of the ROS middleware, enabling efficient development and operation.</p>
    </div>
  </div>

<br>

<h2 id="actuator" style="line-height: 1.2; border-bottom: 1.5px solid #5f3db1; margin-left: 10%;">Actuator</h2>
  <div style="display: flex; align-items: center; gap: 20px;overflow: hidden;" class="container_harp">
    <div style="display: flex; flex-wrap: wrap; justify-content: center; gap: 12px;margin-left: 40%;">
      <div style="background-color:#f0ad4e; color:#ffffff; padding:10px; text-align:center; font-size:20px; font-weight:bold; border-radius:5px; box-shadow: 0px 2px 5px rgba(0, 0, 0, 0.2);">
        <img src="https://img.icons8.com/ios-filled/50/000000/under-construction.png" style="vertical-align: middle; margin-right: 10px; background-color: transparent;">
        Work in progress
        <img src="https://img.icons8.com/ios-filled/50/000000/under-construction.png" style="vertical-align: middle; margin-right: 10px; background-color: transparent;">
      </div>
      <!-- <p style="color: #d0d0d0; line-height: 35px; text-indent: 20px;">Work in progress</p> -->
    </div>
  </div>
