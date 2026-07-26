---
title: HARP project
time: 2024-09-23
---

<div class="harp-logo-wrap">
    <img src="/config/assets/images/HARP/Harp_logo_medallion.webp" alt="HARP logo">
</div>

<div class="harp-title">
  <span class="harp-name">H.A.R.P.</span><br>
  <span class="harp-subtitle">(Holonomic Autonomous Robotic Platform)</span>
</div>

HARP is a long-term personal robotics program focused on the design and integration of complete autonomous robotic systems.

The project covers the full development stack of a robot: mechanical design, custom electronics, embedded software, communication, control, state estimation, autonomous navigation, perception, and high-level decision-making.

Originally created in 2022 for the French Robotics Cup, HARP has progressively evolved from a competition robot into a modular development platform used to experiment with modern robotics architectures and engineering practices.

## Project philosophy

HARP is designed as a complete system. Each generation improves not only individual components, but also how they interact across the robot.

The project prioritizes:

- modular hardware and software interfaces;
- reproducible development environments;
- accurate and observable state estimation;
- robust autonomous navigation;
- maintainable embedded and ROS&nbsp;2 architectures;
- accessible components and manufacturing methods;
- progressive validation on real hardware.

Rather than relying exclusively on expensive robotics hardware, HARP explores how carefully selected off-the-shelf components, custom electronics, open-source software, and modern manufacturing methods can be combined into a capable autonomous platform.

## Platform evolution

### HARP1

HARP1 was the first complete platform and competed in the 2023 French Robotics Cup.

It used ROS&nbsp;1 Melodic, a Raspberry Pi, an Arduino Nano, and rosserial for communication between the high-level computer and embedded electronics. The platform introduced the project’s holonomic drivetrain and provided the first practical experience with complete robot integration, competition constraints, and autonomous match execution.

→ [HARP1 documentation]({{ '/HARP/HARP1/' | relative_url }})

### HARP2

HARP2 introduced a redesigned mechanical and electronic architecture together with a migration to <strong>ROS&nbsp;2 Humble</strong>.

The platform integrates Nav2 for autonomous navigation, ros2_control for hardware abstraction and drivetrain control, and robot_localization for multi-sensor state estimation. Its development focused on improving modularity, odometry accuracy, maintainability, and the overall reliability of the robot.

HARP2 competed in the 2025 and 2026 editions of the French Robotics Cup, providing a practical environment for validating the architecture under real operational constraints.

→ [HARP2 documentation]({{ '/HARP/HARP2/' | relative_url }})

### HARP3

HARP3 is the next major evolution of the platform, built on <strong>ROS&nbsp;2 Lyrical</strong>.

Its architecture is being designed around component composition, modern middleware options, improved state estimation, calibrated holonomic odometry, and more robust navigation and safety systems.

The objective is to create a reusable robotics platform that can serve both as a competition robot and as a foundation for future form factors.

Future generations will progressively explore more complex robotic embodiments, while preserving the same core philosophy: understanding and controlling the entire system, from electronics and embedded firmware to perception, control, and autonomous behaviour.

<div class="harp-video">
    <video controls>
        <source src="/config/assets/images/HARP/video_project_V3_compress.mp4" type="video/mp4">
        Your browser does not support the video tag.
    </video>
</div>
