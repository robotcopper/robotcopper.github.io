---
title: "<img src='/config/assets/images/ROS_logo.png' alt='Logo' style='height: 12px; vertical-align: botom; transform: translateY(0px);'> ROS"
time: 2024-07-17
---

<div style="text-align: center;">
  <img src="/config/assets/images/ROS/ROS_banner.png" alt="ROS banner" style="width: 100%;">
</div>

# ROS&nbsp;2

ROS&nbsp;2 is one of the primary software frameworks used to design, integrate, and operate modern robotic systems.

Like any middleware, ROS&nbsp;2 is not an end in itself but a tool that enables engineers to build complex robotic applications more efficiently. Its purpose is to provide common abstractions, communication mechanisms, and development infrastructure so that teams can focus on robotics rather than repeatedly solving the same software engineering problems.

The ROS ecosystem extends well beyond its core middleware. Projects such as Micro-ROS, Nav2, ros2_control, Gazebo, RViz, MoveIt, PX4, and many others provide mature building blocks for embedded systems, navigation, manipulation, simulation, control, and autonomy. These components are intended to be adapted, extended, or replaced depending on the requirements of the robotic platform.

Whether developing a small mobile robot or a complex humanoid platform, ROS&nbsp;2 provides a common architectural foundation for building modular, maintainable, observable, and reproducible robotic systems. Like any engineering tool, however, its value comes not from using it as-is, but from understanding when to leverage it, customize it, or replace parts of it to meet the needs of the system.

Version compatibility across ROS distros, Ubuntu, and simulators remains a practical concern. The timeline below is a compact map for that — hover to show end-of-life years:

<div class="ros-timeline" tabindex="0" title="Hover to show EOL years">
  <img class="ros-timeline-base" src="/config/assets/images/ROS/ros_timeline.png" alt="ROS timeline">
  <img class="ros-timeline-eol" src="/config/assets/images/ROS/ros_timeline_eol.png" alt="ROS timeline with EOL years">
</div>
<p align="center" style="color:gray; font-size: 0.9rem;">ROS timeline: Ubuntu, Gazebo, and Ignition — also on <a href="https://github.com/robotcopper/ros_timeline">GitHub</a></p>

This section gathers technical articles, implementation notes, reusable components, and lessons learned from developing mobile robots and integrating complex robotic systems.

<div class="info-cards">

  <a class="info-card mint-1" href="{{ '/ROS/behavior_tree/' | relative_url }}">
    <span class="info-card-title">Behavior trees</span>
    <p>
      Implementation notes on structuring robot decisions with behavior trees under ROS&nbsp;2.
    </p>
  </a>

  <a class="info-card mint-2" href="{{ '/ROS/MPPI/' | relative_url }}">
    <span class="info-card-title">MPPI</span>
    <p>
      Notes on Model Predictive Path Integral control in the Nav2 stack.
    </p>
  </a>

  <a class="info-card mint-3" href="{{ '/micro-ROS/' | relative_url }}">
    <span class="info-card-title">Micro-ROS</span>
    <p>
      Bridging microcontrollers into the ROS&nbsp;2 graph — templates and setup notes.
    </p>
  </a>

</div>
