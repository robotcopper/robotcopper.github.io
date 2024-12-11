---
title: HARP1
time: 2024-12-09
---

<div style="display: flex; flex-wrap: wrap; align-items: stretch; background-color: #f0d95c; padding: 10px; border-radius: 10px; font-family: Arial, sans-serif; color: #000; max-width: 92%; box-sizing: border-box; margin: 0 auto; position: relative; overflow: hidden;">
    <!-- Horn effect -->
    <div style="content: ''; position: absolute; top: 0; left: 0; width: 15%; max-width: 90px; aspect-ratio: 1; background: linear-gradient(-45deg, transparent 50%, #d7b845 50%); clip-path: polygon(0 0, 100% 0, 0% 100%);"></div>
    <!-- Text section -->
    <div style="flex: 1 1 60%; margin-right: 20px; min-width: 280px;">
        <center><div style="margin-top: 0; font-size: 36px; font-weight: bold; line-height: 1.2; border-bottom: 1.5px solid #fff; padding-bottom: 5px;">2022-23&nbsp; – &nbsp;<span style="font-family: 'IgnisEtGlaciesSharp'!important; font-size: 42px;">HARP1</span></div></center>
        <center><div style="font-size: 21px; font-weight: bold; margin: 1em 0;">Approved Robot</div></center>
        <p style="padding-left: 25px;">
        <strong>Principle:</strong> The robot was designed to retrieve "cakes" from the field and accumulate them in storage zones. 
        It was also capable of "changing appearance" at the end of the match.
        </p>
        <div style="font-size: 21px; font-weight: bold; margin: 1em 0; padding-left: 25px;">Specifications:</div>
        <ul style="padding-left: 100px;">
            <li>3 Nema 17 stepper motors (59Ncm) for propulsion</li>
            <li>Li-Po 14.8V 8400mAh battery</li>
            <li>1 Arduino Nano V3 microcontroller and 1 Raspberry Pi 3B+ connected via USB</li>
            <li>Chassis made of 3mm threaded rods and laser-cut MDF</li>
            <li>3D-printed mechanical parts</li>
            <li>2D LiDAR for obstacle detection</li>
        </ul>
    </div>
    <!-- Video section -->
    <div style="flex: 1 1 35%; min-width: 280px; display: flex; align-items: stretch;">
        <div style="width: 100%; position: relative;">
            <video style="position: absolute; top: 0; left: 0; width: 100%; height: 100%; background-color: black;" controls>
                <source src="/config/assets/images/HARP/HARP1/HARP1_matchvid2.mp4" type="video/mp4">
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
  <img src="/config/assets/images/HARP/HARP1/HARP1_emblem.png" style="max-width: 250px; max-height: 250px; background-color: transparent; display: block; margin-left: 13%;">
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
      uC & rosserial
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
      <img src="/config/assets/images/HARP/HARP1/HARP1_poster.png" style="background: transparent; border-radius: 20px; width: 50%" >
  </div>
  <p align="center" style="color:#a6a6a6;">Workflow and components of the robot HARP1 (Holonomic Autonomous Robotic Platform 1)</p>

<br>

<h2 id="mechanics" style="line-height: 1.2; border-bottom: 1.5px solid #5f3db1; margin-right: 10%; text-align: -moz-right;">Mechanics</h2>

  <div style="display: flex; align-items: center; gap: 20px;overflow: hidden;" class="container_harp">
    <div style="display: flex; flex-wrap: wrap; justify-content: center; gap: 12px;">
      <p style="color: #d0d0d0; line-height: 35px; text-indent: 20px;">This robot was designed using Onshape (CAD software). Its frame is made of 5mm MDF layers, laser-cut and assembled with 3mm threaded rods. Additional components are made of PLA and 3D-printed. The dimensions comply with the regulations of the French Robotics Cup. This construction method offers excellent adaptability.</p>
    </div>
    <img src="/config/assets/images/HARP/HARP1/Harp1_meca.png" style="max-width: 350px; max-height: 250px; background-color: transparent; display: block; border-radius: 7px;">
  </div>

<h2 id="electronics" style="line-height: 1.2; border-bottom: 1.5px solid #5f3db1; margin-left: 10%;">Electronics</h2>
  <h3 style="font-style: italic; margin-left: 5%;">Power Board</h3>
  <div style="display: flex; align-items: center; gap: 20px; overflow: hidden; max-width: 85%; margin-left: 7%;" class="container_harp">
    <div style="display: grid; grid-template-columns: repeat(2, 1fr); gap: 0px; max-width: 18%; max-height: 18%; border-radius: 3px; overflow: hidden;">
      <img src="/config/assets/images/HARP/HARP1/PawerBoard_PCB.png" style="width: 100%; height: 100%; background-color: transparent;">
      <img src="/config/assets/images/HARP/HARP1/PawerBoard_Schematic.png" style="width: 100%; height: 100%; background-color: transparent;">
      <img src="/config/assets/images/HARP/HARP1/PawerBoard_3D.png" style="width: 100%; height: 100%; background-color: transparent;">
      <img src="/config/assets/images/HARP/HARP1/PawerBoard_Photo.jpg" style="width: 100%; height: 100%; background-color: transparent;">
    </div>
    <div style="display: flex; flex-wrap: wrap; justify-content: center; gap: 12px;">
      <p style="color: #d0d0d0; line-height: 35px; text-indent: 20px;">The power board plays a crucial role in distributing power from the battery to various modules while adapting the electrical parameters as needed. It was designed using KiCad, an open-source software for printed circuit board (PCB) design. Following the validation, the PCB was manufactured. In addition to onboard protections, such as fuses, short-circuit protection, and thermal safeguards, a low-voltage buzzer alarm was added to the battery to enhance its protection.</p>
    </div>
  </div>

  <h3 style="text-align: -moz-right; font-style: italic; margin-right: 5%;">Emergency Stop Button Board</h3>
  <div style="display: flex; align-items: center; gap: 20px; overflow: hidden; max-width: 85%; margin-left: 7%;" class="container_harp">
    <div style="display: flex; flex-wrap: wrap; justify-content: center; gap: 12px;">
      <p style="color: #d0d0d0; line-height: 35px; text-indent: 20px;">The Emergency Stop Button Board ensures the safe operation of the system by integrating a relay controlled by two push buttons. One button serves as the activation switch to power on the system, while the other functions as an emergency stop button, immediately cutting off power when pressed. This board is a critical safety feature, providing a reliable and straightforward way to quickly halt the system in case of unexpected issues. Its design prioritizes simplicity and robustness, ensuring it meets high safety standards.</p>
    </div>
    <div style="display: grid; grid-template-columns: repeat(2, 1fr); gap: 0px; max-width: 18%; max-height: 18%; border-radius: 3px; overflow: hidden;">
      <img src="/config/assets/images/HARP/HARP1/BAU_PCB.png" style="width: 100%; height: 100%; background-color: transparent;">
      <img src="/config/assets/images/HARP/HARP1/BAU_Schematic.png" style="width: 100%; height: 100%; background-color: transparent;">
      <img src="/config/assets/images/HARP/HARP1/BAU_3D.png" style="width: 100%; height: 100%; background-color: transparent;">
      <img src="/config/assets/images/HARP/HARP1/BAU_Photo.jpg" style="width: 100%; height: 100%; background-color: transparent;">
    </div>
  </div>

  <h3 style="font-style: italic; margin-left: 5%;">Limit Switch Management Board</h3>
  <div style="display: flex; align-items: center; gap: 20px; overflow: hidden; max-width: 85%; margin-left: 7%;" class="container_harp">
    <div style="display: grid; grid-template-columns: repeat(2, 1fr); gap: 0px; max-width: 18%; max-height: 18%; border-radius: 3px; overflow: hidden;">
      <img src="/config/assets/images/HARP/HARP1/Limit_switch_mgmt_PCB.png" style="width: 100%; height: 100%; background-color: transparent;">
      <img src="/config/assets/images/HARP/HARP1/Limit_switch_mgmt_Schematic.png" style="width: 100%; height: 100%; background-color: transparent;">
      <img src="/config/assets/images/HARP/HARP1/Limit_switch_mgmt_3D.png" style="width: 100%; height: 100%; background-color: transparent;">
      <img src="/config/assets/images/HARP/HARP1/Limit_switch_mgmt_Photo.jpg" style="width: 100%; height: 100%; background-color: transparent;">
    </div>
    <div style="display: flex; flex-wrap: wrap; justify-content: center; gap: 12px;">
      <p style="color: #d0d0d0; line-height: 35px; text-indent: 20px; text-indent: 20px;">The Limit Switch Management Board was responsible for controlling the matrix of limit switches located at the base of the robot. Comprising 21 interconnected switches arranged in a 3x7 matrix, it operates similarly to a keyboard matrix. This board was designed to support essential functionalities, including system recalibration, detecting the presence of an object precisely where it needs to be manipulated, and triggering an emergency stop in case of unexpected contact. Its implementation enhances the robot's precision, safety, and adaptability to dynamic environments.</p>
    </div>
  </div>

  <h3 style="text-align: -moz-right; font-style: italic; margin-right: 5%;">Stepper Driver Board</h3>
  <div style="display: flex; align-items: center; gap: 20px; overflow: hidden; max-width: 85%; margin-left: 7%;" class="container_harp">
    <div style="display: flex; flex-wrap: wrap; justify-content: center; gap: 12px;">
      <p style="color: #d0d0d0; line-height: 35px; text-indent: 20px;">The Stepper Driver Board integrates A4988 drivers to control the three 59 Ncm NEMA 17 stepper motors. This board acts as the interface between the microcontroller and the motors. It serves as the critical interface between the microcontroller and the motors, translating the PWM control signals from the microcontroller into the necessary power to drive the motors efficiently.</p>
    </div>
    <div style="display: grid; grid-template-columns: repeat(2, 1fr); gap: 0px; max-width: 18%; max-height: 18%; border-radius: 3px; overflow: hidden;">
      <img src="/config/assets/images/HARP/HARP1/StepperDriver_board_PCB.png" style="width: 100%; height: 100%; background-color: transparent;">
      <img src="/config/assets/images/HARP/HARP1/StepperDriver_board_Schematic.png" style="width: 100%; height: 100%; background-color: transparent;">
      <img src="/config/assets/images/HARP/HARP1/StepperDriver_board_3D.png" style="width: 100%; height: 100%; background-color: transparent;">
      <img src="/config/assets/images/HARP/HARP1/StepperDriver_board_Photo.jpg" style="width: 100%; height: 100%; background-color: transparent;">
    </div>
  </div>

  <h3 style="font-style: italic; margin-left: 5%;">Micro Controller Board</h3>
  <div style="display: flex; align-items: center; gap: 20px; overflow: hidden; max-width: 85%; margin-left: 7%;" class="container_harp">
    <div style="display: grid; grid-template-columns: repeat(2, 1fr); gap: 0px; max-width: 18%; max-height: 18%; border-radius: 3px; overflow: hidden;">
      <img src="/config/assets/images/HARP/HARP1/HardwareBoard-Arduino_PCB.png" style="width: 100%; height: 100%; background-color: transparent;">
      <img src="/config/assets/images/HARP/HARP1/HardwareBoard-Arduino_Schematic.png" style="width: 100%; height: 100%; background-color: transparent;">
      <img src="/config/assets/images/HARP/HARP1/HardwareBoard-Arduino_3D.png" style="width: 100%; height: 100%; background-color: transparent;">
      <img src="/config/assets/images/HARP/HARP1/HardwareBoard-Arduino_Photo.jpg" style="width: 100%; height: 100%; background-color: transparent;">
    </div>
    <div style="display: flex; flex-wrap: wrap; justify-content: center; gap: 12px;">
      <p style="color: #d0d0d0; line-height: 35px; text-indent: 20px;">The Micro Controller Board integrates the Arduino Nano V3 and acts as the central hub for controlling various components of the robot. It manages the operation of actuator servo motors, as well as the LED strips. Additionally, this board handles the commands sent to the stepper driver board. </p>
    </div>
  </div>

<br>

<h2 id="micro-controller" style="line-height: 1.2; border-bottom: 1.5px solid #5f3db1; margin-left: 10%;">Micro Controller and Rosserial</h2>

  <div style="display: flex; align-items: center; gap: 20px;overflow: hidden;" class="container_harp">
    <div style="display: flex; flex-wrap: wrap; justify-content: center; gap: 12px;">
      <p style="color: #d0d0d0; line-height: 35px; text-indent: 20px;">Communication between the Arduino Nano V3 and the robot’s central processing unit, the Raspberry Pi 3B+, is facilitated via USB using the rosserial protocol. In this configuration, the Arduino serves as the client, while the Raspberry Pi functions as the server. This setup enables the transmission of motor control commands from the Raspberry Pi to the Arduino, which then generates the appropriate PWM signals to control the motors. <br><br>
      The use of rosserial allows for efficient communication and data exchange between the two devices. This system architecture enables seamless operation of the robot, with the Raspberry Pi handling complex computations and decision-making, while the Arduino executes low-level control tasks, such as motor control.</p>
    </div>
    <img src="/config/assets/images/HARP/HARP1/arduinoxrosserial.png" style="max-width: 350px; max-height: 250px; background-color: transparent; display: block; border-radius: 7px;">
  </div>

<br>

<h2 id="ros" style="line-height: 1.2; border-bottom: 1.5px solid #5f3db1; margin-right: 10%; text-align: -moz-right;">ROS (Robot Operating System)</h2>
  <div style="display: flex; align-items: center; gap: 20px;overflow: hidden;" class="container_harp">
    <img src="/config/assets/images/HARP/HARP1/ros_logo.png" style="max-width: 350px; max-height: 250px; background-color: transparent; display: block; border-radius: 12px;">
    <div style="display: flex; flex-wrap: wrap; justify-content: center; gap: 12px;">
      <p style="color: #d0d0d0; line-height: 35px; text-indent: 20px;"><span style="font-family: 'IgnisEtGlaciesSharp'!important;">HARP1</span> operated under ROS1 Melodic, incorporating the YDlidar-X4 2D LiDAR into its ROS architecture for environmental sensing and obstacle avoidance. A custom Python node was developed to process the lidar data from its ROS driver topic, provide velocity commands, and trigger actuators and LED strips via rosserial. This node acted as the central orchestrator, integrating and executing the robot's game strategy sequence while coordinating various hardware components. This architecture ensured efficient communication and control throughout the system.</p>
    </div>
  </div>

<br>

<h2 id="actuator" style="line-height: 1.2; border-bottom: 1.5px solid #5f3db1; margin-left: 10%;">Actuator</h2>
  <div style="display: flex; align-items: center; gap: 20px;overflow: hidden;" class="container_harp">
    <div style="display: flex; flex-wrap: wrap; justify-content: center; gap: 12px;">
      <p style="color: #d0d0d0; line-height: 35px; text-indent: 20px;"><span style="font-family: 'IgnisEtGlaciesSharp'!important;">HARP1</span> featured three actuators, one on each side of the robot, designed as large paddles to securely hold the game elements—referred to as 'cakes'—during transport. Each actuator was powered by an SG90 9g micro servo motor, which controlled the paddle's rotation angle. These servo motors were driven by PWM signals. The actuators played a crucial role in the robot's ability to interact with and efficiently manipulate the game elements during its tasks.</p>
    </div>
    <img src="/config/assets/images/HARP/HARP1/actuator.gif" style="max-width: 350px; max-height: 250px; background-color: transparent; display: block; border-radius: 12px;">
  </div>