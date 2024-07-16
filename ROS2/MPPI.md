---
title: "<img src='/config/assets/images/NAV2_logo.png' alt='Logo' style='height: 12px; vertical-align: botom; transform: translateY(0px);'> NAV2: MPPI Parameters Tuning"
time: 2024-07-10
---

# MPPI Parameters Tuning in <span style="color:#47c7ef">**NAV2**</span>

<br>

## 1. Introduction

In the world of autonomous robotics, achieving optimal navigation is crucial. <span style="color:#47c7ef">**NAV2**</span> is a powerful framework designed to handle navigation tasks for robots. Within this framework, the **Model Predictive Path Integral (MPPI)** control algorithm plays a significant role. However, to harness the full potential of MPPI, careful tuning of its hyperparameters is essential. This article will explore the importance of parameter tuning for MPPI in <span style="color:#47c7ef">**NAV2**</span> and discuss how practicaly it can be handle.

<br>

## 2. Overview of <span style="color:#47c7ef">**NAV2**</span>

Navigation stack for <span style="color:#4762a6">**ROS 2**</span> also named <span style="color:#47c7ef">**Navigation2 (NAV2)**</span> is an advanced framework for robotic navigation that builds on the capabilities of the <span style="color:#4762a6">**Robot Operating System 2 (ROS 2)**</span>. It offers a comprehensive set of tools and algorithms for path planning, control, and recovery behaviors. MPPI is one of the control algorithms used within <span style="color:#47c7ef">**NAV2**</span> to ensure accurate and efficient navigation.

<span style="color:#47c7ef">**NAV2**</span> is designed to be highly modular, enabling developers to customize and extend its functionalities. Key components of <span style="color:#47c7ef">**NAV2**</span> include global and local planners, controllers, recovery behaviors, and behavior trees. MPPI, as a local control algorithm, is responsible for generating smooth and feasible trajectories for the robot to follow in real-time.

<br>

## 3. Understanding MPPI in <span style="color:#47c7ef">**NAV2**</span>

MPPI is a sampling-based control algorithm that generates a set of potential trajectories and evaluates them based on a cost function. The algorithm then selects the trajectory with the lowest cost, ensuring optimal navigation. The cost function typically considers factors such as distance to the goal, obstacles, control efforts, ect. Taking a look at the paper behind this algorithm may help to understand how it works (specifically for the gamma parameter):

G. Williams et al. "Information-Theoretic Model Predictive Control: Theory and Applications to Autonomous Driving"
  - URL : [https://ieeexplore.ieee.org/document/8558663](https://ieeexplore.ieee.org/document/8558663)
  - PDF : [https://arxiv.org/pdf/1707.02342.pdf](https://arxiv.org/pdf/1707.02342.pdf)


In the context of <span style="color:#47c7ef">**NAV2**</span>, MPPI excels at handling dynamic environments and non-linear dynamics, making it suitable for complex navigation tasks. The algorithm's performance, however, heavily depends on the tuning of its hyperparameters, which control various aspects of the sampling and cost evaluation processes. 

Created by Aleksei Budyakov and adapted & developed for <span style="color:#47c7ef">**NAV2**</span> by Steve Macenski, the aim of MPPI is to become ["the new default controller in NAV2"](https://github.com/ros-navigation/navigation2/issues/2045#issuecomment-1699788228) as it is the most["advanced predictive trajectory planner in the stack"](https://github.com/ros-navigation/navigation2/issues/3664#issuecomment-1611775148).

<br>

## 4. MPPI Controller Parameters Tuning in Humble

### MPPI Parameters Official Description

<table border="1">
  <thead>
    <tr>
      <th>Parameter</th>
      <th>Type</th>
      <th>Default</th>
      <th>Definition</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>motion_model</td>
      <td>string</td>
      <td>DiffDrive</td>
      <td>Type of model [DiffDrive, Omni, Ackermann].<br><br>
        <table border="1">
          <thead>
            <tr>
              <th colspan="4">AckermannConstraints</th>
            </tr>
          </thead>
          <thead>
            <tr>
              <th>Parameter</th>
              <th>Type</th>
              <th>Default</th>
              <th>Definition</th>
            </tr>
          </thead>
          <tbody>
            <tr>
              <td>min_turning_r</td>
              <td>double</td>
              <td>0.2</td>
              <td>Minimum turning radius for ackermann motion model.</td>
            </tr>
          </tbody>
        </table>
      </td>
    </tr>
    <tr>
      <td>critics</td>
      <td>string</td>
      <td>None</td>
      <td>Critics (plugins) names</td>
    </tr>
    <tr>
      <td>iteration_count</td>
      <td>int</td>
      <td>1</td>
      <td>Iteration count in MPPI algorithm. Recommend to keep as 1 and prefer more batches.</td>
    </tr>
    <tr>
      <td>batch_size</td>
      <td>int</td>
      <td>1000</td>
      <td>Count of randomly sampled candidate trajectories</td>
    </tr>
    <tr>
      <td>time_steps</td>
      <td>int</td>
      <td>56</td>
      <td>Number of time steps (points) in each sampled trajectory</td>
    </tr>
    <tr>
      <td>model_dt</td>
      <td>double</td>
      <td>0.05</td>
      <td>Time interval (s) between two sampled points in trajectories.</td>
    </tr>
    <tr>
      <td>prune_distance</td>
      <td>double</td>
      <td>1.5</td>
      <td>Distance ahead of nearest point on path to robot to prune path to.</td>
    </tr>
    <tr>
      <td>vx_std</td>
      <td>double</td>
      <td>0.2</td>
      <td>Sampling standard deviation for VX</td>
    </tr>
    <tr>
      <td>vy_std</td>
      <td>double</td>
      <td>0.2</td>
      <td>Sampling standard deviation for VY</td>
    </tr>
    <tr>
      <td>wz_std</td>
      <td>double</td>
      <td>0.4</td>
      <td>Sampling standard deviation for Wz</td>
    </tr>
    <tr>
      <td>vx_max</td>
      <td>double</td>
      <td>0.5</td>
      <td>Max VX (m/s)</td>
    </tr>
    <tr>
      <td>vy_max</td>
      <td>double</td>
      <td>0.5</td>
      <td>Max VY in either direction, if holonomic. (m/s)</td>
    </tr>
    <tr>
      <td>vx_min</td>
      <td>double</td>
      <td>-0.35</td>
      <td>Min VX (m/s)</td>
    </tr>
    <tr>
      <td>wz_max</td>
      <td>double</td>
      <td>1.9</td>
      <td>Max WZ (rad/s)</td>
    </tr>
    <tr>
      <td>temperature</td>
      <td>double</td>
      <td>0.3</td>
      <td>Selectiveness of trajectories by their costs (The closer this value to 0, the "more" we take in consideration controls with less cost), 0 mean use control with best cost, huge value will lead to just taking mean of all trajectories without cost consideration</td>
    </tr>
    <tr>
      <td>gamma</td>
      <td>double</td>
      <td>0.015</td>
      <td>A trade-off between smoothness (high) and low energy (low). This is a complex parameter that likely won't need to be changed from the default of `0.1` which works well for a broad range of cases. See Section 3D-2 in "Information Theoretic Model Predictive Control: Theory and Applications to Autonomous Driving" for detailed information.</td>
    </tr>
    <tr>
      <td>visualize</td>
      <td>bool</td>
      <td>false</td>
      <td>Publish visualization of trajectories, which can slow down the controller significantly. Use only for debugging. <br><br>
        <table border="1">
          <thead>
            <tr>
              <th colspan="4">TrajectoryVisualizer</th>
            </tr>
          </thead>
          <thead>
            <tr>
              <th>Parameter</th>
              <th>Type</th>
              <th>Default</th>
              <th>Definition</th>
            </tr>
          </thead>
          <tbody>
            <tr>
              <td>trajectory_step</td>
              <td>int</td>
              <td>5</td>
              <td>The step between trajectories to visualize to downsample candidate trajectory pool.</td>
            </tr>
            <tr>
              <td>time_step</td>
              <td>int</td>
              <td>3</td>
              <td>The step between points on trajectories to visualize to downsample trajectory density.</td>
            </tr>
          </tbody>
        </table>
      </td>
    </tr>
    <tr>
      <td>retry_attempt_limit</td>
      <td>int</td>
      <td>1</td>
      <td>Number of attempts to find feasible trajectory on failure for soft-resets before reporting failure.</td>
    </tr>
    <tr>
      <td>regenerate_noises</td>
      <td>bool</td>
      <td>false</td>
      <td>Whether to regenerate noises each iteration or use single noise distribution computed on initialization and reset. Practically, this is found to work fine since the trajectories are being sampled stochastically from a normal distribution and reduces compute jittering at run-time due to thread wake-ups to resample normal distribution.</td>
    </tr>
    <tr>
      <td>max_robot_pose_search_dist</td>
      <td>double</td>
      <td>Costmap half-size</td>
      <td>Max integrated distance ahead of robot pose to search for nearest path point in case of path looping.</td>
    </tr>
    <tr>
      <td>transform_tolerance</td>
      <td>double</td>
      <td>0.1</td>
      <td>Time tolerance for data transformations with TF.</td>
    </tr>
    <tr>
      <td>enforce_path_inversion</td>
      <td>bool</td>
      <td>false</td>
      <td>If true, it will prune paths containing cusping points for segments changing directions (e.g. path inversions) such that the controller will be forced to change directions at or very near the planner's requested inversion point. This is targeting Smac Planner users with feasible paths who need their robots to switch directions where specifically requested.</td>
    </tr>
    <tr>
      <td>inversion_xy_tolerance</td>
      <td>double</td>
      <td>0.2</td>
      <td>Cartesian proximity (m) to path inversion point to be considered "achieved" to pass on the rest of the path after path inversion.</td>
    </tr>
    <tr>
      <td>inversion_yaw_tolerance</td>
      <td>double</td>
      <td>0.4</td>
      <td>Angular proximity (radians) to path inversion point to be considered "achieved" to pass on the rest of the path after path inversion. 0.4 rad = 23 deg.</td>
    </tr>
  </tbody>
</table>

<br>

### Tuning

<table border="0" style="border: 3px solid #00AA80; border-collapse: collapse;" >
  <thead>
    <tr>
      <th>Parameter</th>
      <th>Tuning</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>motion_model</td>
      <td>Has to suit your robot's motion model.<br><br>
        <table border="1">
          <thead>
            <tr>
              <th colspan="4">AckermannConstraints</th>
            </tr>
          </thead>
          <tbody>
            <tr>
              <td>min_turning_r</td>
              <td>Cannot be set to 0 to handle turning-in-place behavior (the value can be low but not zero).</td>
            </tr>
          </tbody>
        </table></td>
    </tr>
    <tr>
      <td>critics</td>
      <td>
        It's crucial to choose the right critics to use, and not all critics can be used with all motion models, here are the minimum critics for each model: <br><br>
        - DiffDrive: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; ["GoalCritic", "GoalAngleCritic", "ObstaclesCritic", "PathAngleCritic", "PathFollowCritic", "PreferForwardCritic"]<br>
        - Omni: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; ["GoalCritic", "GoalAngleCritic", "ObstaclesCritic", "TwirlingCritic", "PathFollowCritic", "PreferForwardCritic"]<br>
        - Ackermann: &nbsp;["GoalCritic", "GoalAngleCritic", "ObstaclesCritic", "PathAngleCritic", "PathFollowCritic", "PreferForwardCritic"]<br><br>
        These critics were determined by looking at the <a href="https://github.com/ros-navigation/navigation2/blob/3dc1b51f5275fff73ca360a541116755a7b84f1e/nav2_mppi_controller/benchmark/controller_benchmark.cpp#L131">controller benchmark</a> in the MPPI plugin code.
      </td>
    </tr>
    <tr>
      <td>iteration_count</td>
      <td>Can be left at <code>1</code>.</td>
    </tr>
    <tr>
      <td>batch_size</td>
      <td>Can be left at <code>1000</code> (1000 at 50 Hz or 2000 at 30 Hz seems to produce good results).</td>
    </tr>
    <tr>
      <td>time_steps</td>
      <td>Must be adjusted to match model_dt and prune_distance constraints.</td>
    </tr>
    <tr>
      <td>model_dt</td>
      <td>Should be set to the duration of your control frequency. So if your control frequency is 20hz, this should be 0.05. However, you may also set it lower but not larger.<br>
      Length of each time step’s <code>dt</code> timestep, in seconds. <code>time_steps * model_dt</code> is the prediction horizon.
      </td>
    </tr>
    <tr>
      <td>prune_distance</td>
      <td>Reduce the length of the path to be followed by the robot, by eliminating parts of the path that are too far away to be relevant or necessary at any given time. This parameter relates to the prediction of the physical range of the trajectory, while the prediction horizon relates to the temporal range of the trajectory.<br>
      I recommend keeping this parameter within the same order of magnitude as the prediction horizon, or at least proportional to it.<br><br>
      Warning: this parameter is critical, a prune_distance too low compared to the desired speed will result in a truncated speed. prune_distance must therefore remain proportional to the target speed and prediction horizon.
      </td>
    </tr>
    <tr>
      <td>vx_std</td>
      <td rowspan="3">Indicates the extent to which VX, VY and WZ values may differ from their population average when MPPI is looking for a trajectory to follow. A value that is too low will not give the algorithm enough freedom and result in no trajectory being followed, and a value that is too high may result in non-optimal trajectories being chosen.<br> 
      A good tip is to leave them at their default values.</td>
    </tr>
    <tr>
      <td>vy_std</td>
    </tr>
    <tr>
      <td>wz_std</td>
    </tr>
    <tr>
      <td>vx_max</td>
      <td>Target speed for the DiffDrive and Ackermann motion models and X speed target for the Omni motion models</td>
    </tr>
    <tr>
      <td>vy_max</td>
      <td>+/-Y speed target for the Omni motion models</td>
    </tr>
    <tr>
      <td>vx_min</td>
      <td>Target speed when reversing</td>
    </tr>
    <tr>
      <td>wz_max</td>
      <td>Rotational target speed</td>
    </tr>
    <tr>
      <td>temperature</td>
      <td>Can be left at <code>0.3</code>.</td>
    </tr>
    <tr>
      <td>gamma</td>
      <td>
        For more detailed information, I recommend referring to the <a href="https://arxiv.org/pdf/1707.02342.pdf">associated research paper</a>.<br>
        In the vast majority of cases, this parameter can remain at <code>0.015</code>.
      </td>
    </tr>
    <tr>
      <td>visualize</td>
      <td>Useful to activate when setting MPPI parameters. <br><br>
        <table border="1">
          <thead>
            <tr>
              <th colspan="2">TrajectoryVisualizer</th>
            </tr>
          </thead>
          <tbody>
            <tr>
              <td>trajectory_step</td>
              <td rowspan="2">The higher the value, the fewer paths will be displayed. <br> 
              I advise you to display only the main path when setting the parameters, by setting both parameters to <code>100</code> for example.</td>
            </tr>
            <tr>
              <td>time_step</td>
            </tr>
          </tbody>
        </table>
      </td>
    </tr>
    <tr>
      <td>retry_attempt_limit</td>
      <td>Can be left at <code>1</code>.</td>
    </tr>
    <tr>
      <td>regenerate_noises</td>
      <td>Can be left at <code>false</code>.</td>
    </tr>
    <tr>
      <td>max_robot_pose_search_dist</td>
      <td>Can remain unconfigured</td>
    </tr>
    <tr>
      <td>transform_tolerance</td>
      <td>Common parameters, you can manage it as you are used to. Can be left at <code>0.1</code>.</td>
    </tr>
    <tr>
      <td>enforce_path_inversion</td>
      <td rowspan="3">Make driving in reverse predominant over driving forwards. To be enabled if this is the desired behavior.</td>
    </tr>
    <tr>
      <td>inversion_xy_tolerance</td>
    </tr>
    <tr>
      <td>inversion_yaw_tolerance</td>
    </tr>
  </tbody>
</table>

<br>

## 6. MPPI Critics Tuning in Humble

### Constraint Critic
#### Official Description

 | Parameter   | Type   | Default | Definition                      |
 | ----------- | ------ | ------- | ------------------------------- |
 | cost_weight | double | 4.0     | Weight to apply to critic term. |
 | cost_power  | int    | 1       | Power order to apply to term.   |

#### Tuning

Depending on the motion model's kinematics, this critic determines whether the trajectory speed will comply with the maximum and minimum limits requested and penalises any deviation. Score function <a href="https://github.com/ros-navigation/navigation2/blob/12a9c1d805847709e3b82f8dcfbb43c67b5b2937/nav2_mppi_controller/src/critics/constraint_critic.cpp#L41">here</a>.

<table border="0" style="border: 3px solid #00AA80; border-collapse: collapse;" >
  <tr>
    <th>Parameter</th>
    <th>Tuning</th>
  </tr>
  <tr>
    <td>cost_weight</td>
    <td>Adjusts the relative influence of the critic in the overall cost calculation. A higher weight makes the critic more influential, which increases its <strong>impact</strong> on optimisation decisions.</td>
  </tr>
  <tr>
    <td>cost_power</td>
    <td> Affects the <strong>sensitivity</strong> of the cost compared with variations in the critic. </td>
  </tr>
</table>

### Goal Angle Critic
#### Official Description

 | Parameter             | Type   | Default | Definition                      |
 | --------------------- | ------ | ------- | ------------------------------- |
 | cost_weight           | double | 3.0     | Weight to apply to critic term. |
 | cost_power            | int    | 1       | Power order to apply to term.   |
 | threshold_to_consider | double | 0.5     | Minimal distance between robot and goal above which angle goal cost considered.                                                        |

#### Tuning

This critic enables you to control the extent to which the orientation of the robot must coincide with the orientation of the target. <br>
In my experience, the weight of this critic is generally one of the last to be modified if the final orientation is not to your liking. Please note: you must first be satisfied with the GoalChecker's <code>yaw_goal_tolerance</code> params in the controller.

<table border="0" style="border: 3px solid #00AA80; border-collapse: collapse;" >
  <tr>
    <th>Parameter</th>
    <th>Tuning</th>
  </tr>
  <tr>
    <td>cost_weight</td>
    <td>Adjusts the relative influence of the critic in the overall cost calculation. A higher weight makes the critic more influential, which increases its <strong>impact</strong> on optimisation decisions.</td>
  </tr>
  <tr>
    <td>cost_power</td>
    <td> Affects the <strong>sensitivity</strong> of the cost compared with variations in the critic. </td>
  </tr>
  <tr>
    <td>threshold_to_consider</td>
    <td>Should be set at Path Angle Critic's <code>threshold_to_consider</code> for a smooth transition.</td>
  </tr>
</table>

### Goal Critic
#### Official Description

 | Parameter             | Type   | Default | Definition                      |
 | --------------------- | ------ | ------- | ------------------------------- |
 | cost_weight           | double | 5.0     | Weight to apply to critic term. |
 | cost_power            | int    | 1       | Power order to apply to term.   |
 | threshold_to_consider | double | 1.4     | Distance between robot and goal above which goal cost starts being considered                                                             |

#### Tuning   

This critic enables you to control the extent to which the position of the robot must coincide with the position of the target. <br>
In my experience, the weight of this critic is generally one of the last to be modified if the final position is not to your liking. Please note: you must first be satisfied with the GoalChecker's <code>xy_goal_tolerance</code> params in the controller.


<table border="0" style="border: 3px solid #00AA80; border-collapse: collapse;" >
  <tr>
    <th>Parameter</th>
    <th>Tuning</th>
  </tr>
  <tr>
    <td>cost_weight</td>
    <td>Adjusts the relative influence of the critic in the overall cost calculation. A higher weight makes the critic more influential, which increases its <strong>impact</strong> on optimisation decisions.</td>
  </tr>
  <tr>
    <td>cost_power</td>
    <td> Affects the <strong>sensitivity</strong> of the cost compared with variations in the critic. </td>
  </tr>
  <tr>
    <td>threshold_to_consider</td>
    <td>Can be set at prediction horizon for a smooth transition to the path-following critic.</td>
  </tr>
</table>

### Path Angle Critic
#### Official Description

 | Parameter             | Type   | Default | Definition                      |
 | --------------------- | ------ | ------- | ------------------------------- |
 | cost_weight           | double | 2.0     | Weight to apply to critic term. |
 | cost_power            | int    | 1       | Power order to apply to term.   |
 | threshold_to_consider | double | 0.5     | Distance between robot and goal above which path angle cost stops being considered                                                       |
 | offset_from_furthest  | int    | 4       | Number of path points after furthest one any trajectory achieves to compute path angle relative to.                                  |
 | max_angle_to_furthest | double | 1.2     | Angular distance between robot and goal above which path angle cost starts being considered                                           |
 | forward_preference    | bool   | true    | Whether or not your robot has a preference for which way is forward in motion. Different from if reversing is generally allowed, but if you robot contains *no* particular preference one way or another.                                    |

#### Tuning

This critic enables you to control the extent to which the orientation of the robot must coincide with the local upcoming orientation of the path generated by the planner. <br>

<table border="0" style="border: 3px solid #00AA80; border-collapse: collapse;" >
  <tr>
    <th>Parameter</th>
    <th>Tuning</th>
  </tr>
  <tr>
    <td>cost_weight</td>
    <td>Adjusts the relative influence of the critic in the overall cost calculation. A higher weight makes the critic more influential, which increases its <strong>impact</strong> on optimisation decisions.</td>
  </tr>
  <tr>
    <td>cost_power</td>
    <td> Affects the <strong>sensitivity</strong> of the cost compared with variations in the critic. </td>
  </tr>
  <tr>
    <td>threshold_to_consider</td>
    <td>Ideally, for a smooth transition, it should be defined at the distance of <code>offset_from_furthest</code>. Therefore, I suggest starting by setting it slightly greater than the <code>prune_distance</code>, and then adjusting it according to the desired behavior if necessary.</td>
  </tr>
  <tr>
    <td>offset_from_furthest</td>
    <td>furthest point = point on the path where at least one of the MPPI trajectories has approached furthest. Defined<a href="https://github.com/ros-navigation/navigation2/blob/12a9c1d805847709e3b82f8dcfbb43c67b5b2937/nav2_mppi_controller/include/nav2_mppi_controller/tools/utils.hpp#L310"> here</a>. <code>offset_from_furthest</code> is the number of points after the furthest one that will be considerated to estimate the angle. The higher the value, the less the angular variation of the path will affect the MPPI trajectory. <br>
    Note that this point is therefore dependent on the length of the paths generated by MPPI, in other words on the <code>prune_distance</code>.</td>
  </tr>
  <tr>
    <td>max_angle_to_furthest</td>
    <td>Threshold angle at which critic is activated. In other words, the robot is not realigned within this tolerance angle.</td>
  </tr>
  <tr>
    <td>forward_preference</td>
    <td>Different from the reversing parameter, in the case where there is no constraint, will favour or not the forward orientation.</td>
  </tr>
</table>

### Path Align Critic
#### Official Description

 | Parameter                | Type   | Default | Definition                      |
 | ------------------------ | ------ | ------- | ------------------------------- |
 | cost_weight              | double | 10.0    | Weight to apply to critic term. |
 | cost_power               | int    | 1       | Power order to apply to term.   |
 | threshold_to_consider    | double | 0.5     | Distance between robot and goal above which path align cost stops being considered                                                     |
 | offset_from_furthest     | int    | 20      | Checks that the candidate trajectories are sufficiently far along their way tracking the path to apply the alignment critic. This ensures that path alignment is only considered when actually tracking the path, preventing awkward initialization motions preventing the robot from leaving the path to achieve the appropriate heading.                 |
 | trajectory_point_step    | int    | 4       | Step of trajectory points to evaluate for path distance to reduce compute time. Between 1-10 is typically reasonable.                      |
 | max_path_occupancy_ratio | double | 0.07(7%)| Maximum proportion of the path that can be occupied before this critic is not considered to allow the obstacle and path follow critics to avoid obstacles while following the path's intent in presence of dynamic objects in the scene.        |
 | use_path_orientations    | bool   | false   | Whether to consider path's orientations in path alignment, which can be useful when paired with feasible smac planners to incentivize directional changes only where/when the smac planner requests them. If you want the robot to deviate and invert directions where the controller sees fit, keep as false. If your plans do not contain orientation information (e.g. navfn), keep as false.                                                                  |

#### Tuning

This critic enables you to control the extent to which the position of the robot must coincide with the position of the path generated by the planner based on the costmap. Indeed, the aim is to minimise deviations from the path. <br>
Note that this critic must be balanced with the deviating critics, Cost Critic or Obstacles Critic, who will tend to stray from the path.<br>

<table border="0" style="border: 3px solid #00AA80; border-collapse: collapse;" >
  <tr>
    <th>Parameter</th>
    <th>Tuning</th>
  </tr>
  <tr>
    <td>cost_weight</td>
    <td>Adjusts the relative influence of the critic in the overall cost calculation. A higher weight makes the critic more influential, which increases its <strong>impact</strong> on optimisation decisions.</td>
  </tr>
  <tr>
    <td>cost_power</td>
    <td> Affects the <strong>sensitivity</strong> of the cost compared with variations in the critic. </td>
  </tr>
  <tr>
    <td>threshold_to_consider</td>
    <td>May be set at Goal Critic's <code>threshold_to_consider</code> for a smooth transition.</td>
  </tr>
  <tr>
    <td>offset_from_furthest</td>
    <td>furthest point = point on the path where at least one of the MPPI trajectories has approached furthest. Defined<a href="https://github.com/ros-navigation/navigation2/blob/12a9c1d805847709e3b82f8dcfbb43c67b5b2937/nav2_mppi_controller/include/nav2_mppi_controller/tools/utils.hpp#L310"> here</a>. <code>offset_from_furthest</code> is the number of points after the furthest one that will be considerated to estimate that the path is followed. The higher the value, the less the deviations from the path will affect the MPPI trajectory. <br>
    Note that this point is therefore dependent on the length of the paths generated by MPPI, in other words on the <code>prune_distance</code>.</td>
  </tr>
  <tr>
    <td>trajectory_point_step</td>
    <td>Specifies the step of trajectory points to evaluate for path distance. This reduces computational effort by evaluating path alignment at spaced intervals rather than every single point.</td>
  </tr>
  <tr>
    <td>max_path_occupancy_ratio</td>
    <td>Sets the maximum proportion of the path that can be occupied by obstacles before this critic is no longer considered. This allows the algorithm to focus on path-following even in dynamic environments, adjusting its behavior based on obstacle presence. The higher the value, the later the avoidance.</td>
  </tr>
  <tr>
    <td>use_path_orientations</td>
    <td>If set to true, the algorithm considers changes in path orientation, potentially influencing directional adjustments based on the planner's guidance. <br>
    If set to false, it adheres strictly to the path's intended trajectory without incorporating directional changes based on orientation cues.</td>
  </tr>  
</table>

### Path Follow Critic
#### Official Description

 | Parameter             | Type   | Default | Definition                      |
 | ---------------       | ------ | ------- |-------------------------------- |
 | cost_weight           | double | 5.0     | Weight to apply to critic term. |
 | cost_power            | int    | 1       | Power order to apply to term.   |
 | offset_from_furthest  | int    | 6       | Number of path points after furthest one any trajectory achieves to drive path tracking relative to.                                 |
 | threshold_to_consider | float  | 1.4     | Distance between robot and goal above which path follow cost stops being considered                                                       | 
 
#### Tuning

This critic is designed to encourage the robot to choose actions that move it along the planned trajectory towards its goal. This means that the system favours movements that progress in the direction of the goal rather than sideways or away from it.

<table border="0" style="border: 3px solid #00AA80; border-collapse: collapse;" >
  <tr>
    <th>Parameter</th>
    <th>Tuning</th>
  </tr>
  <tr>
    <td>cost_weight</td>
    <td>Adjusts the relative influence of the critic in the overall cost calculation. A higher weight makes the critic more influential, which increases its <strong>impact</strong> on optimisation decisions.</td>
  </tr>
  <tr>
    <td>cost_power</td>
    <td> Affects the <strong>sensitivity</strong> of the cost compared with variations in the critic. </td>
  </tr>
  <tr>
    <td>offset_from_furthest</td>
    <td>furthest point = point on the path where at least one of the MPPI trajectories has approached furthest. Defined<a href="https://github.com/ros-navigation/navigation2/blob/12a9c1d805847709e3b82f8dcfbb43c67b5b2937/nav2_mppi_controller/include/nav2_mppi_controller/tools/utils.hpp#L310"> here</a>. <code>offset_from_furthest</code> is the number of points after the furthest one that will be considerated to estimate that the path is followed. The higher the value, the less the deviations from the path will affect the MPPI trajectory. <br>
    Note that this point is therefore dependent on the length of the paths generated by MPPI, in other words on the <code>prune_distance</code>.</td>
  </tr>
  <tr>
    <td>threshold_to_consider</td>
    <td>Can be set at prediction horizon for a smooth transition with goal critics.</td>
  </tr>
</table>

### Prefer Forward Critic
#### Official Description

 | Parameter             | Type   | Default | Definition                      |
 | --------------------- | ------ | ------- |-------------------------------- |
 | cost_weight           | double | 5.0     | Weight to apply to critic term. |
 | cost_power            | int    | 1       | Power order to apply to term.   |
 | threshold_to_consider | double | 0.5     | Distance between robot and goal above which prefer forward cost stops being considered                                                  |

#### Tuning

This critic incentivizes moving in the forward direction, rather than reversing.

<table border="0" style="border: 3px solid #00AA80; border-collapse: collapse;" >
  <tr>
    <th>Parameter</th>
    <th>Tuning</th>
  </tr>
  <tr>
    <td>cost_weight</td>
    <td>Adjusts the relative influence of the critic in the overall cost calculation. A higher weight makes the critic more influential, which increases its <strong>impact</strong> on optimisation decisions.</td>
  </tr>
  <tr>
    <td>cost_power</td>
    <td> Affects the <strong>sensitivity</strong> of the cost compared with variations in the critic. </td>
  </tr>
  <tr>
    <td>threshold_to_consider</td>
    <td>Can be set at Goal Angle Critic's <code>threshold_to_consider</code> for a smooth transition.</td>
  </tr>
</table>

### Twirling Critic
#### Official Description

 | Parameter   | Type   | Default | Definition                      |
 | ----------- | ------ | ------- | ------------------------------- |
 | cost_weight | double | 10.0    | Weight to apply to critic term. |
 | cost_power  | int    | 1       | Power order to apply to term.   |

#### Tuning

This criticism only concerns the <code>Omni</code> model, to avoid getting a spinning top robot when the track is followed.

<table border="0" style="border: 3px solid #00AA80; border-collapse: collapse;" >
  <tr>
    <th>Parameter</th>
    <th>Tuning</th>
  </tr>
  <tr>
    <td>cost_weight</td>
    <td>Adjusts the relative influence of the critic in the overall cost calculation. A higher weight makes the critic more influential, which increases its <strong>impact</strong> on optimisation decisions.</td>
  </tr>
  <tr>
    <td>cost_power</td>
    <td> Affects the <strong>sensitivity</strong> of the cost compared with variations in the critic. </td>
  </tr>
</table>

### Velocity Deadband Critic
#### Official Description

 | Parameter           | Type     | Default         | Definition                      |
 | ------------------- | ------   | --------------- |-------------------------------- |
 | cost_weight         | double   | 35.0            | Weight to apply to critic term. |
 | cost_power          | int      | 1               | Power order to apply to term.   |
 | deadband_velocities | double[] | [0.0, 0.0, 0.0] | The array of deadband velocities [vx, vz, wz]. A zero array indicates that the critic will take no action.                                 |

#### Tuning

Since MPPI includes speed and acceleration limits, the velocity_smoother has become practically irrelevant. The only thing missing is the deadband and with this Velocity Deadband Critic we can now get rid of the velocity_smoother.


### Cost Critic and Obstacles Critic

Cost Critic and Obstacles Critic can be difficult to differentiate and knowing which of these two avoidance critics to choose can be confusing. <br>
Based on the <a href="https://github.com/ros-navigation/navigation2/issues/4057#issue-2088411659"> original Cost Critic issue</a>, this is what can be learned from it:

<table border="0" style="border: 3px solid #00AA80; border-collapse: collapse;" >
  <tr>
    <th>Critic</th>
    <th>Advantages</th>
    <th>Disadvantages</th>
  </tr>
  <tr>
    <td><strong>Obstacles Critic</strong></td>
    <td>
      <ul>
        <li>Fine distance evaluation: provides a precise assessment of collision risks.</li>
        <li>Detailed collision management: granular risk management for close obstacles.</li>
        <li>Parameter flexibility: allows fine-tuning for behavior around obstacles.</li>
      </ul>
    </td>
    <td>
      <ul>
        <li>Increased complexity: adds algorithmic complexity with distance calculations from costs.</li>
        <li>Higher computational load: estimating distances and footprint inflation increases computational load.</li>
        <li>Quantization effect: cost-to-distance conversion may introduce quantization effects.</li>
        <li>Inscribed area precision issue: score remains the same regardless of the exact distance to the obstacle.</li>
      </ul>
    </td>
  </tr>
  <tr>
    <td><strong>Cost Critic (proposed InflationCostCritic)</strong></td>
    <td>
      <ul>
        <li>Simplicity: directly uses cost values, simplifying the calculation.</li>
        <li>Efficiency: reduces computational load by avoiding complex conversions.</li>
        <li>Adaptive behavior: flexible in narrow and wide spaces due to the exponential factor.</li>
        <li>Easier tuning: fewer parameters to adjust, reducing the risk of quantization effects.</li>
      </ul>
    </td>
    <td>
      <ul>
        <li>Less precision: does not account for precise distances to obstacles.</li>
        <li>Fixed critical score: less granular risk management.</li>
        <li>Dependence on costmap quality: performance relies on the quality and resolution of the costmap.</li>
      </ul>
    </td>
  </tr>
</table>

With that in mind, it's up to you to decide which of these critics is best suited to your use case. One point to note is that in the event of a narrow zone crossing, Cost Critic seems more likely to generate centred navigation between obstacles than Obstacles Critic.

#### Cost Critic
##### Official Description

 | Parameter            | Type   | Default   | Definition                                         |
 | -------------------- | ------ | --------- |--------------------------------------------------- |
 | consider_footprint   | bool   | False     | Whether to use point cost (if robot is circular or low compute power) or compute SE2 footprint cost. |
 | cost_weight          | double | 3.81      | Wight to apply to critic to avoid obstacles.       | 
 | cost_power           | int    | 1         | Power order to apply to term.                      |
 | collision_cost       | double | 1000000.0 | Cost to apply to a true collision in a trajectory. |
 | critical_cost        | double | 300.0     | Cost to apply to a pose with any point in in inflated space to prefer distance from obstacles.                                                               |
 | near_goal_distance   | double | 0.5       | Distance near goal to stop applying preferential obstacle term to allow robot to smoothly converge to goal pose in close proximity to obstacles.           |
 | inflation_layer_name | string | ""        | Name of the inflation layer. If empty, it uses the last inflation layer in the costmap. If you have multiple inflation layers, you may want to specify the name of the layer to use.                                                                                |

#### Obstacles Critic
##### Official Description

 | Parameter                 | Type   | Default | Definition                             |
 | ------------------------- | ------ | ------- | -------------------------------------- |
 | consider_footprint        | bool   | False   | Whether to use point cost (if robot is circular or low compute power) or compute SE2 footprint cost.                                           |
 | critical_weight           | double | 20.0    | Weight to apply to critic for near collisions closer than `collision_margin_distance` to prevent near collisions **only** as a method of virtually inflating the footprint. This should not be used to generally influence obstacle avoidance away from critical collisions.                                                                             |
 | repulsion_weight          | double | 1.5     | Weight to apply to critic for generally preferring routes in lower cost space. This is separated from the critical term to allow for fine tuning of obstacle behaviors with path alignment for dynamic scenes without impacting actions which may directly lead to near-collisions. This is applied within the `inflation_radius` distance from obstacles. |
 | cost_power                | int    | 1       | Power order to apply to term.          |
 | collision_cost            | double | 10000.0 | Cost to apply to a true collision in a trajectory.                                                                             |
 | collision_margin_distance | double | 0.10    | Margin distance from collision to apply severe penalty, similar to footprint inflation. Between 0.05-0.2 is reasonable.                         |
 | near_goal_distance        | double | 0.5     | Distance near goal to stop applying preferential obstacle term to allow robot to smoothly converge to goal pose in close proximity to obstacles.   
 | cost_scaling_factor       | double | 10.0    | Exponential decay factor across inflation radius. This should be the same as for your inflation layer (Humble only)                            |
 | inflation_radius          | double | 0.55    | Radius to inflate costmap around lethal obstacles. This should be the same as for your inflation layer (Humble only)                            |

<br>

## 8. Conclusion

Tuning is a critical step in optimizing the performance of the MPPI controller in <span style="color:#4762a6">**ROS 2**</span> <span style="color:#47c7ef">**NAV2**</span>. By carefully adjusting parameters, significant improvements in navigation efficiency and robustness can be achieved.  

<br>

```note
These guidelines given here are based solely on my experience and understanding of the MPPI and in no circumstances replace the official documentation from which I have drawn inspiration and which I quote in the following references. 
```

<br>

## 9. References

- [NAV2 Documentation.](https://docs.nav2.org/configuration/packages/configuring-mppic.html)
- [NAV2_mppi_controller github](https://github.com/ros-navigation/navigation2/tree/humble/nav2_mppi_controller)
- [NAV2_mppi_controller github Notes to Users](https://github.com/ros-navigation/navigation2/tree/humble/nav2_mppi_controller#notes-to-users)

These references provide comprehensive information on configuring the MPPI controller in the Navigation2 stack, including detailed documentation, source code, and user notes, which form the basis of this discussion.

<br>