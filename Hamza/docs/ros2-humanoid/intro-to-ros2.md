---
title: Introduction to ROS 2 for Physical AI
sidebar_position: 1
description: Understanding what ROS 2 is and why it matters for humanoid robots, including DDS concepts
---

# Introduction to ROS 2 for Physical AI

## Learning Objectives
- Understand what ROS 2 is and its role in robotics
- Explain why ROS 2 matters specifically for humanoid robotics
- Describe the core concepts of DDS (Data Distribution Service) and its importance in ROS 2
- Identify the key differences between ROS 1 and ROS 2

## Prerequisites
- Basic programming knowledge (Python or C++)
- Understanding of fundamental robotics concepts
- Familiarity with Linux command line (helpful but not required)

## What is ROS 2?

Robot Operating System 2 (ROS 2) is not an operating system, but rather a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

ROS 2 provides:
- Distributed computing between processes
- Package management
- Standardized robot data structures (messages)
- Device drivers
- Simulation environments
- Visualization tools

## Why ROS 2 Matters for Humanoids

Humanoid robots present unique challenges that make ROS 2 particularly valuable:

### Coordination Complexity
Humanoid robots have many degrees of freedom (DOFs) - typically 20+ joints. Each joint requires control, and these controls must be coordinated. ROS 2's distributed architecture allows different control systems to communicate seamlessly.

### Sensor Integration
Humanoids use numerous sensors (IMUs, cameras, force/torque sensors, etc.). ROS 2's message-passing system makes it easy to share sensor data between different processing nodes.

### Behavior Architecture
Humanoids need complex behavior architectures that can handle multiple tasks simultaneously (walking, grasping, speaking, etc.). ROS 2's node-based system allows for modular, reusable behavior components.

## DDS Concepts in ROS 2

Data Distribution Service (DDS) is the middleware that powers ROS 2's communication. Understanding DDS concepts is crucial for effective ROS 2 development:

### Publishers and Subscribers
The fundamental communication pattern in ROS 2 is publisher-subscriber. Publishers send messages without knowing who will receive them, and subscribers receive messages without knowing who sent them.

### Topics
Topics are named buses over which nodes exchange messages. A node can publish messages to a topic, and other nodes can subscribe to that topic to receive those messages.

### Services
Services provide a request/reply communication pattern. A client sends a request to a service and waits for a response.

### Quality of Service (QoS)
DDS provides Quality of Service settings that allow you to control how messages are delivered, including reliability, durability, and liveliness.

## Key Differences from ROS 1

ROS 2 addresses several limitations of ROS 1:

- **Real-time support**: ROS 2 can work with real-time systems, crucial for robot control
- **Multi-robot systems**: Better support for coordinating multiple robots
- **Security**: Built-in security features
- **DDS middleware**: Pluggable middleware architecture for different performance and platform needs

## Code Examples

### Simple Publisher Example

Here's a basic example of a ROS 2 publisher in Python:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Simple Subscriber Example

And here's a corresponding subscriber:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary
- ROS 2 is a flexible framework for writing robot software
- It's particularly valuable for humanoid robots due to coordination complexity and sensor integration needs
- DDS is the middleware that powers ROS 2's communication
- Key concepts include publishers/subscribers, topics, services, and QoS settings
- ROS 2 offers improvements over ROS 1 in real-time support, multi-robot systems, and security

## Next Steps

For more information on how ROS 2 components communicate, see the [ROS 2 Communication Model](./communication-model.md) chapter.

## Assessment
import Assessment from '@site/src/components/Assessment';

<Assessment
  title="Introduction to ROS 2 Quiz"
  questions={[
    {
      text: "What does DDS stand for in the context of ROS 2?",
      options: ["Distributed Data System", "Data Distribution Service", "Dynamic Data Sharing", "Distributed Development System"],
      correctAnswer: 1
    },
    {
      text: "Which communication pattern involves a request/reply model?",
      options: ["Publisher-Subscriber", "Client-Service", "Peer-to-Peer", "Broadcast"],
      correctAnswer: 1
    },
    {
      text: "Why is ROS 2 particularly valuable for humanoid robots?",
      options: [
        "Simple control requirements",
        "Coordination complexity and sensor integration needs",
        "Low computational requirements",
        "Single sensor usage"
      ],
      correctAnswer: 1
    }
  ]}
/>

## Next Steps

For more information on how ROS 2 components communicate, see the [ROS 2 Communication Model](./communication-model.md) chapter.

## Resources

- [Official ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [DDS Concepts](https://fast-dds.docs.eprosima.com/en/v2.6.0/fastdds/dds_layer/dds_layer.html)

## Navigation

[Next: ROS 2 Communication Model](./communication-model.md)