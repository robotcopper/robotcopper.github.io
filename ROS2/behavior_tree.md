---
title : Behavior tree with NAV2
time: 2024-05-29
---

<br>

In robotics, efficient navigation is paramount for successful task completion, whether it's guiding a robot through a cluttered environment or navigating complex terrain. The <span style="color:#47c7ef">**Navigation2 (NAV2)**</span> framework within the <span style="color:#4762a6">**Robot Operating System (ROS)**</span> provides a powerful suite of tools for robot navigation. One key component in orchestrating robot behavior is the <span style="color:#9cff71">**Behavior Tree (BT)**</span>, a hierarchical control structure widely used for decision-making in autonomous systems which in <span style="color:#47c7ef">**NAV2**</span> is responsible for navigation decisions from point A to point B.

This tutorial focuses on integrating custom <span style="color:#9cff71">**Behavior Trees**</span> leaf into <span style="color:#47c7ef">**NAV2**</span> <span style="color:#9cff71">**BT**</span>, leveraging the flexibility and modularity of the <span style="color:#9cff71">**BT**</span> architecture to enhance robot navigation strategies. By combining the structured decision-making of <span style="color:#9cff71">**BTs**</span> with the robust navigation capabilities of <span style="color:#47c7ef">**NAV2**</span>, robots can exhibit more intelligent and adaptive behavior in diverse environments.

<br>

## Understanding Behavior Trees (BTs) in Robotics

<span style="color:#9cff71">**Behavior Trees**</span> are the successors of **state machines** and provide a hierarchical representation of robot behavior, organizing actions, conditions, and control flow nodes into a tree structure. At the root of the tree is the main decision-making node, which delegates tasks to child nodes based on predefined conditions and priorities. This hierarchical organization facilitates clear, modular, and scalable behavior design, making <span style="color:#9cff71">**BTs**</span> a popular choice in robotics.

In the context of <span style="color:#47c7ef">**NAV2**</span>, <span style="color:#9cff71">**Behavior Trees**</span> serve as the control mechanism for guiding robot navigation. By defining navigation-related tasks and conditions within the <span style="color:#9cff71">**BT**</span> structure, robots can autonomously plan and execute navigation behaviors while dynamically adapting to changes in the environment.

<br>

## Integrating Behavior Trees with NAV2

To integrate <span style="color:#9cff71">**Behavior Trees**</span> with <span style="color:#47c7ef">**NAV2**</span>, we utilize tools such as <span style="color:#95764f">**Groot**</span>. <span style="color:#95764f">**Groot**</span>, a graphical tool for editing <span style="color:#9cff71">**BTs**</span>, simplifies the design and visualization of complex behavior hierarchies. Custom nodes, tailored to specific navigation tasks or conditions, extend the functionality of <span style="color:#47c7ef">**NAV2**</span> and enhance the robot's decision-making capabilities.

This tutorial provides step-by-step instructions for:

1. **Setting up Groot for editing NAV2 Behavior Trees.**
2. **Adding custom nodes to the NAV2 Behavior Tree framework.**
3. **Modifying Humble configuration files to incorporate custom nodes into the navigation pipeline.**

<br>

By following these steps, developers can extend the capabilities of <span style="color:#47c7ef">**NAV2**</span> and tailor robot behavior to specific navigation challenges, ultimately improving the overall autonomy and performance of robotic systems.

Now, let's dive into the practical implementation of integrating <span style="color:#9cff71">**Behavior Trees**</span> with <span style="color:#47c7ef">**NAV2**</span>, starting with setting up the necessary tools and environment.

<br>

## Using groot

- download and install [groot2](https://www.behaviortree.dev/groot/):
<br>
[```https://s3.us-west-1.amazonaws.com/download.behaviortree.dev/groot2_linux_installer/Groot2-v1.5.2-linux-installer.run```](https://s3.us-west-1.amazonaws.com/download.behaviortree.dev/groot2_linux_installer/Groot2-v1.5.2-linux-installer.run)
		
- add the path to your bashrc:
```bash
export PATH=$PATH:<path_to_groot2>/Groot2/bin
```
	
- Now groot can be launched from a terminal by doing:
```bat
$ groot2
```

- use groot on a BT NAV2:
you must first load the NAV2 nodes by opening ```/opt/ros/humble/share/nav2_behavior_tree/nav2_tree_nodes.xml```
		
- Once this is done, you can open any BT that depends on these nodes.

<div style="display: flex; justify-content: center;">
    <video width="854" height="480" controls>
        <source src="/config/assets/images/BT_nav2/groot_tuto.mp4" type="video/mp4">
        Your browser does not support the video tag.
    </video>
</div>

<br>

## Adding a custom node to NAV2 BT in Humble
	
In Humble, modifying a NAV2 BT has not yet been made obvious, so here's what we're interested in: adding a custom node.
In the Navigation2 stack, we will have to act on the nav2_behavior_tree and nav2_bt_navigator packages.
	
Let's consider adding a condition node called GoalDistanceCondition:
	
- in the ***nav2_behavior_tree*** package we will add these files:
	
```	
nav2_behavior_tree
├── include
│   └── nav2_behavior_tree
│       └── plugins
│           └── condition
│               └── goal_distance_condition.hpp
└── plugins
    └── condition
        └── goal_distance_condition.cpp
```
- still in the ***nav2_behavior_tree*** package, we are going to modify these files:
	
```	
nav2_behavior_tree
├── CMakeLists.txt
└── nav2_tree_nodes.xml
```
- by adding these two lines to the ***CMakeLists.txt***:
	
```cmake
add_library(nav2_goal_distance_condition_bt_node SHARED plugins/condition/goal_distance_condition.cpp)
list(APPEND plugin_libs nav2_goal_distance_condition_bt_node)
```

- and in ***nav2_tree_nodes.xml*** by adding your custom node ID. For this exemple:
	
```xml  
<Condition ID="GoalDistance">
    <input_port name="goal">Destination</input_port>
    <input_port name="global_frame">Reference frame</input_port>
    <input_port name="robot_base_frame">Robot base frame</input_port>
    <input_port name="threshold">threshold</input_port>
</Condition>
```

- we then need to modify this file in the ***nav2_bt_navigator*** package

```	
nav2_bt_navigator
└── src
    └── bt_navigator.cpp
```
- and add the following line in *const std::vector<std::string> plugin_libs*:
	
```cpp
"nav2_goal_distance_condition_bt_node"
```
<img src="/config/assets/images/BT_nav2/bt_navigator_lib_list.png" style="background: transparent;">

The customised node is now added and practically ready to use. The only thing left to do is to add it to the *.yaml* navigation configuration file generally called *nav2_params.yaml* the following line to the list of plugins used by the *bt_navigator*:

```yml
- nav2_goal_distance_condition_bt_node
```

<img src="/config/assets/images/BT_nav2/bt_navigator_param.png" style="background: transparent;">


Below you will find the **hpp** and **cpp** code mentioned and you can use it as a basis for the development of your own nodes:
**_goal_distance_condition.hpp_ content:**

<details markdown="1">
```cpp
#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__GOAL_DISTANCE_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__GOAL_DISTANCE_CONDITION_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_behavior_tree
{

class GoalDistanceCondition : public BT::ConditionNode
{
public:
GoalDistanceCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

GoalDistanceCondition() = delete;

~GoalDistanceCondition() override;

BT::NodeStatus tick() override;

void initialize();

bool isGoalReached();

static BT::PortsList providedPorts()
{
    return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination"),
    BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
    BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame"),
    BT::InputPort<double>("threshold", 0.5, "Threshold distance")
    };
}

protected:
void cleanup()
{}

private:
rclcpp::Node::SharedPtr node_;
std::shared_ptr<tf2_ros::Buffer> tf_;

bool initialized_;
double threshold_;
double goal_reached_tol_;
std::string global_frame_;
std::string robot_base_frame_;
double transform_tolerance_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__GOAL_DISTANCE_CONDITION_HPP_
```
</details>



**_goal_distance_condition.cpp_ content:**

<details markdown="1">

```cpp
#include <string>
#include <memory>

#include "nav2_util/robot_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/condition/goal_distance_condition.hpp"

namespace nav2_behavior_tree
{

GoalDistanceCondition::GoalDistanceCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  initialized_(false),  
  global_frame_("map"),
  robot_base_frame_("base_link")
{
  getInput("threshold", threshold_);
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
}


GoalDistanceCondition::~GoalDistanceCondition()
{
  cleanup();
}

BT::NodeStatus GoalDistanceCondition::tick()
{
  if (!initialized_) {
    initialize();
  }

  BT::NodeStatus status = isGoalReached() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;

  // Display node status in console
  // std::string status_str = (status == BT::NodeStatus::SUCCESS) ? "SUCCESS" : "FAILURE";
  // std::cout << "GoalDistanceCondition tick() status: " << status_str << std::endl;

  return status;
}
void GoalDistanceCondition::initialize()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  node_->get_parameter("threshold", threshold_);

  nav2_util::declare_parameter_if_not_declared(
    node_, "goal_reached_tol",
    rclcpp::ParameterValue(0.25));
  node_->get_parameter_or<double>("goal_reached_tol", goal_reached_tol_, 0.25);
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  node_->get_parameter("transform_tolerance", transform_tolerance_);

  initialized_ = true;
}

bool GoalDistanceCondition::isGoalReached()
{
  geometry_msgs::msg::PoseStamped current_pose;

  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    return false;
  }

  geometry_msgs::msg::PoseStamped goal;
  getInput("goal", goal);

  auto travelle = nav2_util::geometry_utils::euclidean_distance(
    goal.pose, current_pose.pose);
  // std::cout << "travelle : " << travelle << std::endl;
  
  return travelle <= threshold_ ;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::GoalDistanceCondition>(name, config);
    };

  factory.registerBuilder<nav2_behavior_tree::GoalDistanceCondition>("GoalDistance", builder);
}
```
</details>