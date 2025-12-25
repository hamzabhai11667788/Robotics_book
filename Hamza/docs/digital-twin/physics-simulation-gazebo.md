---
title: Physics Simulation with Gazebo
sidebar_position: 1
description: Understanding physics-based simulation with Gazebo for humanoid robotics
---

# Physics Simulation with Gazebo

## Learning Objectives
- Understand the fundamentals of Gazebo physics simulation
- Learn how to create and configure humanoid models in Gazebo
- Implement realistic physics parameters for accurate simulation
- Apply best practices for physics-based humanoid simulation

## Prerequisites
- Basic understanding of ROS/ROS2 concepts
- Familiarity with robotics simulation concepts
- Knowledge of 3D coordinate systems and transformations

## Introduction to Gazebo Physics

Gazebo is a powerful physics simulation engine that provides realistic robot simulation capabilities. It is widely used in robotics research and development for testing algorithms and robot designs in a safe, cost-effective environment.

### Key Features of Gazebo Physics
- Realistic physics simulation using ODE, Bullet, or Simbody engines
- High-fidelity sensor simulation (LiDAR, cameras, IMU, etc.)
- Support for complex environments and multi-robot scenarios
- Integration with ROS/ROS2 for seamless development workflows

## Setting up a Basic Gazebo Environment

To get started with Gazebo physics simulation:

1. **Install Gazebo**: Ensure you have a compatible version of Gazebo installed along with ROS/ROS2
2. **Create a world file**: Define your simulation environment using SDF (Simulation Description Format)
3. **Load your robot model**: Use URDF (Unified Robot Description Format) to define your robot

### Basic World File Structure

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Your robot model will be inserted here -->
    
  </world>
</sdf>
```

## Physics Parameters for Realistic Simulation

To achieve realistic simulation, several physics parameters need to be carefully configured:

### Gravity
The default gravity setting is usually sufficient (9.8 m/s²), but it can be adjusted for different environments:

```xml
<world>
  <gravity>0 0 -9.8</gravity>
  <!-- Physics engine configuration -->
  <physics name="default_physics" type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
  </physics>
</world>
```

### Friction Properties
Proper friction coefficients are essential for realistic contact simulation:

```xml
<collision name="collision">
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>
        <mu2>1.0</mu2>
      </ode>
    </friction>
  </surface>
</collision>
```

## Creating Humanoid Models in Gazebo

Humanoid robots require special attention to achieve stable and realistic simulation. Here are key considerations:

### Joint Configuration
Humanoid joints need appropriate limits and damping to mimic human-like movement:

```xml
<joint name="hip_joint" type="revolute">
  <parent>torso</parent>
  <child>thigh</child>
  <limit effort="30" velocity="1.0" lower="-1.57" upper="1.57"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

### Mass Distribution
Realistic mass distribution is critical for stable simulation:

- Torso: 20-30 kg
- Thigh: 5-8 kg
- Shank: 3-5 kg
- Foot: 1-2 kg

## Best Practices for Physics Simulation

### Time Step Considerations
- Use smaller time steps for more accurate simulation (typically 0.001s)
- Balance accuracy with computational efficiency
- Monitor real-time factor to ensure simulation runs at desired speed

### Contact Simulation
- Configure appropriate contact properties for stable interactions
- Use SDF contact parameters to define material properties
- Test different ODE parameters to achieve desired behavior

### Sensor Integration
- Place sensors accurately on the robot model
- Configure sensor noise parameters to match real hardware
- Verify sensor data validity during simulation

## Troubleshooting Common Issues

### Instability in Simulation
- Check mass and inertia properties of links
- Verify joint limits and damping parameters
- Adjust physics engine parameters (time step, solver iterations)

### Penetration Between Objects
- Increase constraint force mixing (CFM) and reduce error reduction parameter (ERP)
- Verify collision geometry is properly defined
- Check for overlapping collision meshes

## Summary
- Gazebo provides a robust physics simulation environment for humanoid robots
- Proper configuration of physics parameters is essential for realistic simulation
- Humanoid models require special attention to joint limits and mass distribution
- Best practices include appropriate time step selection and contact parameter tuning

## Next Steps

For creating digital twins and human-robot interaction, see the [Digital Twins & HRI in Unity](./digital-twins-hri-unity.md) chapter.

## Assessment
import Assessment from '@site/src/components/Assessment';

<Assessment
  title="Physics Simulation with Gazebo Quiz"
  questions={[
    {
      text: "What is the default gravity setting in Gazebo?",
      options: ["5.8 m/s²", "9.8 m/s²", "12.8 m/s²", "6.8 m/s²"],
      correctAnswer: 1
    },
    {
      text: "Which physics engines does Gazebo support?",
      options: ["ODE only", "ODE, Bullet, and Simbody", "Bullet only", "ODE and Bullet"],
      correctAnswer: 1
    },
    {
      text: "What is an appropriate time step for accurate physics simulation?",
      options: ["0.1s", "0.01s", "0.001s", "0.05s"],
      correctAnswer: 2
    }
  ]}
/>

## Resources

- [Official Gazebo Documentation](http://gazebosim.org/)
- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [ROS with Gazebo](http://wiki.ros.org/gazebo)

## Navigation

[Next: Digital Twins & HRI in Unity](./digital-twins-hri-unity.md)