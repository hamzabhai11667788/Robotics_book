---
title: ROS 2 Communication Model
sidebar_position: 2
description: Understanding Nodes, Topics, Services, and basic rospy-based agent and controller flow in ROS 2
---

# ROS 2 Communication Model

## Learning Objectives
- Understand the fundamental concepts of Nodes, Topics, and Services in ROS 2
- Implement a basic rospy-based agent and controller
- Describe the communication flow between different components
- Apply communication patterns in humanoid robotics applications

## Prerequisites
- Understanding of basic ROS 2 concepts (covered in the previous chapter)
- Basic Python programming knowledge
- Familiarity with the publisher-subscriber and client-service patterns

## Nodes in ROS 2

A node is an executable that uses ROS 2 to communicate with other nodes. Nodes are the fundamental building blocks of a ROS 2 system. Each node is designed to perform a specific task and communicate with other nodes to achieve more complex behaviors.

### Creating a Node

In ROS 2, a node is typically implemented as a class that inherits from `rclpy.node.Node`. Here's a basic example:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Node initialization code here

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics in ROS 2

Topics are named buses over which nodes exchange messages. The publisher-subscriber pattern is the most common communication method in ROS 2. Publishers send messages to a topic, and subscribers receive messages from that topic.

### Publishers and Subscribers

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher.publish(msg)

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')
```

## Services in ROS 2

Services provide a request/reply communication pattern. A client sends a request to a service and waits for a response. This is useful when you need a specific response to a request.

### Creating Services

```python
# Server
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {response.sum}')
        return response

# Client
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

    def send_request(self, a, b):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.future = self.cli.call_async(request)
```

## Basic rospy-based Agent and Controller Flow

In humanoid robotics, agents and controllers often follow specific communication patterns to achieve coordinated behavior.

### Agent Implementation

An agent in ROS 2 typically consists of:
1. Sensors (subscribers) that gather information about the environment
2. Decision-making logic that processes sensor data
3. Actuators (publishers) that execute actions

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

class HumanoidAgent(Node):
    def __init__(self):
        super().__init__('humanoid_agent')
        
        # Subscribe to sensor data
        self.sensor_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
        
        # Publish commands to actuators
        self.command_pub = self.create_publisher(
            JointTrajectory,
            'joint_trajectory',
            10)
        
        # Agent logic timer
        self.timer = self.create_timer(0.1, self.agent_logic)
        
        self.joint_positions = {}
    
    def joint_state_callback(self, msg):
        # Update joint positions from sensor data
        for name, position in zip(msg.name, msg.position):
            self.joint_positions[name] = position
    
    def agent_logic(self):
        # Decision-making logic goes here
        # For example, move to a target position
        if 'left_leg_joint' in self.joint_positions:
            target_pos = self.joint_positions['left_leg_joint'] + 0.1
            self.move_joint('left_leg_joint', target_pos)
    
    def move_joint(self, joint_name, target_position):
        # Create and publish trajectory command
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [joint_name]
        # Add trajectory points...
        self.command_pub.publish(traj_msg)
```

### Controller Implementation

A controller typically:
1. Receives commands from agents
2. Translates high-level commands to low-level motor commands
3. Provides feedback on execution status

```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import JointTrajectoryControllerState

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')
        
        # Subscribe to trajectory commands
        self.command_sub = self.create_subscription(
            JointTrajectory,
            'joint_trajectory',
            self.trajectory_callback,
            10)
        
        # Publish controller state
        self.state_pub = self.create_publisher(
            JointTrajectoryControllerState,
            'controller_state',
            10)
        
        # Controller logic
        self.current_positions = {}
    
    def trajectory_callback(self, msg):
        # Execute trajectory command
        for point in msg.points:
            # Send commands to hardware
            self.execute_trajectory_point(msg.joint_names, point)
    
    def execute_trajectory_point(self, joint_names, point):
        # Implementation of trajectory execution
        # This would interface with the actual hardware
        pass
```

## Communication Patterns in Humanoid Robotics

Humanoid robots require complex communication patterns due to their multiple subsystems:

### Hierarchical Control
- High-level planning nodes communicate with mid-level controllers
- Mid-level controllers coordinate with low-level actuator nodes

### Sensor Fusion
- Multiple sensor nodes publish data to a central fusion node
- The fusion node processes all sensor data into a coherent world model

### Behavior Coordination
- Behavior nodes coordinate to ensure actions don't conflict
- For example, walking and grasping behaviors need to coordinate

## Summary
- Nodes are the fundamental building blocks of ROS 2 systems
- Topics enable publisher-subscriber communication patterns
- Services provide request/reply communication
- Agents and controllers follow specific patterns in humanoid robotics
- Effective communication is crucial for coordinating complex humanoid behaviors

## Next Steps

To learn about representing robot structure in ROS 2, see the [Robot Structure with URDF](./robot-structure-urdf.md) chapter.

## Assessment
import Assessment from '@site/src/components/Assessment';

<Assessment
  title="ROS 2 Communication Model Quiz"
  questions={[
    {
      text: "What is the primary communication pattern for continuous data streams in ROS 2?",
      options: ["Client-Service", "Publisher-Subscriber", "Peer-to-Peer", "Broadcast"],
      correctAnswer: 1
    },
    {
      text: "Which pattern is most appropriate for requesting a specific computation with a guaranteed response?",
      options: ["Publisher-Subscriber", "Client-Service", "Action", "Parameter Server"],
      correctAnswer: 1
    },
    {
      text: "In humanoid robotics, what do agents typically do?",
      options: [
        "Only publish sensor data",
        "Gather sensor data, make decisions, and send actuator commands",
        "Only execute low-level commands",
        "Store historical data only"
      ],
      correctAnswer: 1
    }
  ]}
/>

## Next Steps

To learn about representing robot structure in ROS 2, see the [Robot Structure with URDF](./robot-structure-urdf.md) chapter.

## Resources

- [ROS 2 Topics and Services](https://docs.ros.org/en/humble/Concepts/About-Topics.html)
- [ROS 2 Client Libraries](https://docs.ros.org/en/humble/Concepts/About-Client-Libraries.html)
- [ROS 2 Quality of Service](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)

## Navigation

[Previous: Introduction to ROS 2](./intro-to-ros2.md) | [Next: Robot Structure with URDF](./robot-structure-urdf.md)