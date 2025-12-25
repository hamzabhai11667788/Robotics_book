---
title: Robot Structure with URDF
sidebar_position: 3
description: Understanding URDF for humanoid robots and simulation readiness
---

# Robot Structure with URDF

## Learning Objectives
- Understand the fundamentals of Unified Robot Description Format (URDF)
- Create URDF descriptions for humanoid robots
- Implement URDF for simulation readiness
- Apply URDF best practices for humanoid robotics

## Prerequisites
- Basic understanding of ROS 2 concepts (covered in previous chapters)
- Knowledge of 3D coordinate systems and transformations
- Basic understanding of robot kinematics

## URDF Fundamentals

Unified Robot Description Format (URDF) is an XML format for representing a robot model. URDF files describe the physical and visual properties of a robot, including:

- **Links**: Rigid parts of the robot (e.g., chassis, arm segments)
- **Joints**: Connections between links that allow motion
- **Visual elements**: How the robot appears in simulation
- **Collision elements**: How the robot interacts with its environment
- **Inertial properties**: Mass, center of mass, and inertia tensor for physics simulation

### Basic URDF Structure

A basic URDF file has the following structure:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links define rigid parts of the robot -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Joints define connections between links -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0.2 0" rpy="0 0 0"/>
  </joint>

  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## URDF for Humanoid Robots

Humanoid robots have complex kinematic structures that require careful URDF design. Key considerations include:

### Kinematic Chains
Humanoid robots typically have multiple kinematic chains:
- Left arm chain (from torso to left hand)
- Right arm chain (from torso to right hand)
- Left leg chain (from torso to left foot)
- Right leg chain (from torso to right foot)
- Spine and head chain

### Joint Types
Different joint types are used for different robot parts:
- **Revolute**: Single axis rotation with limits (e.g., elbow)
- **Continuous**: Single axis rotation without limits (e.g., wheel)
- **Prismatic**: Single axis translation (e.g., linear actuator)
- **Fixed**: No movement (e.g., sensor mount)
- **Floating**: 6 DOF (rarely used)
- **Planar**: Motion on a plane (rarely used)

### Example Humanoid URDF Structure

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </visual>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="2.36" effort="10" velocity="1"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Simulation Readiness

For a humanoid robot model to be simulation-ready, it must meet several requirements:

### Proper Kinematic Structure
- All links must be connected through joints
- No disconnected components
- Clear parent-child relationships forming a tree structure (or kinematic loops if applicable)

### Physics Properties
- Accurate mass properties for each link
- Proper inertial tensors
- Realistic collision geometry

### Visual Properties
- Appropriate visual geometry for rendering
- Materials and textures if needed
- Proper scaling

### Control Interface
- Joint limits that reflect real hardware capabilities
- Appropriate joint types for intended motion
- Integration with ROS 2 control systems

## URDF Best Practices

### File Organization
- Break complex models into multiple files using `<xacro>` or `<include>`
- Use consistent naming conventions
- Separate kinematic, visual, and collision properties appropriately

### Xacro for Complex Models
Xacro (XML Macros) allows you to create parameterized URDF models:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_xacro">
  <xacro:property name="M_PI" value="3.14159"/>

  <!-- Define a macro for a simple arm segment -->
  <xacro:macro name="arm_segment" params="name parent_link length radius">
    <joint name="${name}_joint" type="revolute">
      <parent link="${parent_link}"/>
      <child link="${name}_link"/>
      <origin xyz="0 0 -${length}" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="-${M_PI/2}" upper="${M_PI/2}" effort="10" velocity="1"/>
    </joint>

    <link name="${name}_link">
      <visual>
        <geometry>
          <cylinder length="${length}" radius="${radius}"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <cylinder length="${length}" radius="${radius}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="1"/>
        <origin xyz="0 0 -${length/2}"/>
        <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Use the macro to create arm segments -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </visual>
  </link>

  <xacro:arm_segment name="upper_arm" parent_link="torso" length="0.3" radius="0.05"/>
  <xacro:arm_segment name="lower_arm" parent_link="upper_arm_link" length="0.3" radius="0.05"/>
</robot>
```

## URDF Tools and Validation

Several tools help with URDF development and validation:

### Robot State Publisher
- Publishes transforms for all links in the robot
- Required to visualize the robot in RViz

### Joint State Publisher
- Publishes joint states for visualization
- Useful for testing without actual hardware

### URDF Validation
- Use `check_urdf` command to validate URDF structure
- Use `urdf_to_graphiz` to visualize the kinematic tree

## Summary
- URDF describes robot structure using links and joints
- Humanoid robots require complex kinematic chains
- Simulation readiness requires proper physics and visual properties
- Xacro helps manage complex URDF models
- Tools exist for validating and visualizing URDF models

## Next Steps

To learn more about official ROS 2 documentation and resources, see the links in the next section.

## Assessment
import Assessment from '@site/src/components/Assessment';

<Assessment
  title="URDF for Humanoid Robots Quiz"
  questions={[
    {
      text: "What does URDF stand for?",
      options: [
        "Unified Robot Definition Format",
        "Universal Robot Description Format",
        "Unified Robot Description Format",
        "Universal Robot Design Framework"
      ],
      correctAnswer: 2
    },
    {
      text: "Which joint type allows rotation without limits?",
      options: ["Revolute", "Prismatic", "Fixed", "Continuous"],
      correctAnswer: 3
    },
    {
      text: "What is the purpose of the <inertial> tag in URDF?",
      options: [
        "Define visual appearance",
        "Specify collision properties",
        "Define physics properties for simulation",
        "Set joint limits"
      ],
      correctAnswer: 2
    }
  ]}
/>

## Resources

- [URDF/XML Format](https://wiki.ros.org/urdf/XML)
- [Xacro](https://wiki.ros.org/xacro)
- [Robot State Publisher](https://wiki.ros.org/robot_state_publisher)

## Navigation

[Previous: ROS 2 Communication Model](./communication-model.md)