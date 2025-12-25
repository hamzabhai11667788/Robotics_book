---
title: Nav2 for Humanoid Navigation
sidebar_position: 3
description: Understanding Navigation2 for humanoid robot navigation and path planning in simulated and real environments
---

# Nav2 for Humanoid Navigation

## Learning Objectives
- Understand the Navigation2 (Nav2) framework for humanoid robot navigation
- Configure Nav2 for humanoid-specific navigation requirements
- Implement path planning algorithms suitable for humanoid robots
- Integrate Nav2 with perception systems for autonomous navigation
- Apply best practices for humanoid robot navigation in complex environments

## Prerequisites
- Understanding of ROS 2 concepts (covered in previous chapters)
- Basic knowledge of path planning and navigation concepts
- Familiarity with Isaac Sim and Isaac ROS (from previous chapters)
- Knowledge of coordinate frames and transformations

## Introduction to Navigation2 (Nav2)

Navigation2 (Nav2) is the state-of-the-art navigation framework for ROS 2, designed to provide reliable and robust path planning and navigation capabilities for mobile robots. For humanoid robots, Nav2 requires special configuration to account for the unique kinematic and dynamic properties of bipedal locomotion systems.

### Key Components of Nav2

Nav2 consists of several key components that work together to provide navigation capabilities:

- **Navigator**: Coordinates the navigation process
- **Planner Server**: Global path planning component
- **Controller Server**: Local path following and obstacle avoidance
- **Recovery Server**: Recovery behaviors for challenging situations
- **BT Navigator**: Behavior Tree-based navigation orchestrator
- **Lifecycle Manager**: Manages the lifecycle of navigation components

### Nav2 Architecture for Humanoid Robots

Humanoid robots present unique navigation challenges that require specialized Nav2 configuration:

- **Kinematic constraints**: Bipedal locomotion differs significantly from wheeled robots
- **Stability considerations**: Navigation paths must account for balance and stability
- **Step/obstacle negotiation**: Ability to navigate stairs, curbs, and obstacles
- **Dynamic balancing**: Real-time adjustments during locomotion

## Configuring Nav2 for Humanoid Robots

Setting up Nav2 for humanoid robots requires careful consideration of the robot's physical properties and locomotion capabilities.

### Base Configuration

```yaml
# Basic Nav2 configuration for humanoid robot
bt_navigator:
  ros__parameters:
    # Behavior tree to use for navigation
    bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    # Node that logs results
    default_server_timeout: 20
    # Goal checker plugin
    goal_checker_plugin: "goal_checker"
    # Goal checker parameters
    goal_checker:
      plugin: "nav2_goal_checker::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

planner_server:
  ros__parameters:
    # Global planner plugin
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

controller_server:
  ros__parameters:
    # Controller plugins
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_mppi_controller::MppiController"
      # Humanoid-specific parameters
      time_steps: 20
      control_freq: 20
      horizon_dt: 1.0
      reference_heading_weight: 1.0
      reference_speed_weight: 0.0
      obstacle_cost_weight: 1.0
      goal_dist_cost_weight: 1.0
      goal_angle_cost_weight: 0.5
      xy_goal_tolerance: 0.1
      trans_stopped_velocity: 0.05
      short_circuit_trajectory: true

# Recovery behaviors
recoveries_server:
  ros__parameters:
    recovery_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_recoveries/Spin"
    backup:
      plugin: "nav2_recoveries/BackUp"
    wait:
      plugin: "nav2_recoveries/Wait"

# Local costmap
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_roll_pitch: false
      # Humanoid-specific parameters
      robot_radius: 0.3  # Larger than typical wheeled robots
      resolution: 0.05
      footprint_padding: 0.01
      inflation_radius: 0.55
      cost_scaling_factor: 3.0
      map_topic: map
      always_send_full_costmap: true
  local_costmap_client:
    ros__parameters:
      robot_base_frame: base_link
      transform_tolerance: 0.3
      use_sim_time: true
  local_costmap_rclcpp_node:
    ros__parameters:
      transform_tolerance: 0.3

# Global costmap
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_roll_pitch: false
      # Humanoid-specific parameters
      robot_radius: 0.3
      resolution: 0.1
      footprint_padding: 0.01
      inflation_radius: 1.0
      cost_scaling_factor: 3.0
      map_topic: map
      always_send_full_costmap: true
  global_costmap_client:
    ros__parameters:
      robot_base_frame: base_link
      transform_tolerance: 0.3
      use_sim_time: true
  global_costmap_rclcpp_node:
    ros__parameters:
      transform_tolerance: 0.3