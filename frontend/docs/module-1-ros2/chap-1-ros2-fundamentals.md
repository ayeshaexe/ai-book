---
title: ROS 2 Fundamentals
sidebar_label: Chapter 1 - ROS 2 Fundamentals
---

# Chapter 1: ROS 2 Fundamentals

## Introduction

This chapter introduces the core concepts of ROS 2 (Robot Operating System 2), which serves as the middleware for robotic applications. ROS 2 provides a framework for developing robot applications, offering tools for communication, hardware abstraction, device drivers, and libraries for implementing common robot functionality. Understanding these fundamentals is crucial as they form the foundation for all subsequent topics in this module.

## Concepts

### What is ROS 2?

ROS 2 is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

Key characteristics of ROS 2:
- Open-source robotics middleware
- Provides services such as hardware abstraction, device drivers, libraries, and more
- Supports distributed computing for multiple processes across multiple machines
- Offers rich development tools for testing, visualization, and debugging

### ROS 2 Architecture

ROS 2 uses a distributed architecture where different components (nodes) communicate with each other through a publish-subscribe model or service-based communication. This architecture enables:
- Decoupling of robot software components
- Reusability of robot software components
- Distributed computation across multiple machines
- Language independence

### Nodes, Topics, and Services

The three fundamental communication concepts in ROS 2 are:

#### Nodes
A node is a process that performs computation. Nodes are the fundamental building blocks of a ROS 2 program. Multiple nodes are managed by a ROS 2 graph, which coordinates the communication between them.

In practice, a node is typically a single process that performs a specific task, such as sensor data processing, path planning, or actuator control.

#### Topics
Topics are named buses over which nodes exchange messages. Topics implement a publish/subscribe communication pattern where multiple nodes can publish to or subscribe from the same topic.

The publish/subscribe pattern allows for asynchronous communication between nodes. Publishers send messages to a topic without knowing who will receive them, and subscribers receive messages from a topic without knowing where they came from.

#### Services
Services implement a request/reply communication pattern. In this pattern, a node sends a request message to another node and waits for a reply message.

Services are synchronous, meaning the client node waits for the server node to process the request and return a response before continuing execution.

## Examples

### Node Example

Here's a minimal example of a ROS 2 node written in Python:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Minimal node created')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()

    # Keep the node alive
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Publisher/Subscriber Example

Here's an example of publisher and subscriber nodes:

**Publisher:**
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
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
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

**Subscriber:**
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
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Example

Here's an example of service client and server:

**Service Server:**
```python
from add_two_ints_srv.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

This chapter covered the fundamental concepts of ROS 2, including its architecture and purpose. You now understand the basic building blocks of ROS 2 systems:
- Nodes as the fundamental computational units
- Topics for asynchronous publish/subscribe communication
- Services for synchronous request/reply communication

These concepts form the foundation for all subsequent topics in this module. The examples provided demonstrate practical implementations of these concepts using Python and rclpy, which will be explored in more detail in the next chapter.

## Advanced Concepts

### Lifecycle Nodes

ROS 2 introduces lifecycle nodes to manage the state of complex systems. Lifecycle nodes have a well-defined state machine that includes states like unconfigured, inactive, active, and finalized. This allows for coordinated startup, shutdown, and error recovery in complex robotic systems.

### Actions

In addition to topics and services, ROS 2 provides actions for long-running tasks that may take significant time to complete. Actions combine the features of topics and services, providing feedback during execution, goal preemption, and result reporting.

### Quality of Service (QoS) Settings

ROS 2 provides Quality of Service (QoS) settings that allow you to fine-tune communication characteristics. QoS settings include reliability, durability, liveliness, and deadline policies that can be configured based on your application's requirements.

### Parameters

Parameters in ROS 2 allow nodes to be configured at runtime. They provide a way to change node behavior without recompiling code. Parameters can be declared, set, and retrieved dynamically, and they support various data types.

## Real-World Applications

ROS 2 is used in a variety of applications including:

- Autonomous vehicles and mobile robots
- Industrial automation and manufacturing
- Service robots for healthcare and hospitality
- Research platforms for AI and robotics
- Agricultural and construction robotics
- Space and underwater exploration robots

## Best Practices

When working with ROS 2, consider the following best practices:

1. **Modularity**: Design your system with well-defined interfaces between components
2. **Naming Conventions**: Use consistent naming for topics, services, and parameters
3. **Error Handling**: Implement robust error handling and recovery mechanisms
4. **Testing**: Create comprehensive tests for your nodes and systems
5. **Documentation**: Document your nodes, topics, and services clearly
6. **Resource Management**: Properly manage resources and clean up when shutting down

## Troubleshooting Common Issues

### Communication Issues
- Verify that nodes are on the same ROS domain ID
- Check that topic names match exactly (including case sensitivity)
- Ensure network configuration allows multicast communication

### Performance Issues
- Monitor CPU and memory usage of your nodes
- Consider QoS settings for time-sensitive applications
- Use appropriate message types for your data

### Debugging Strategies
- Use `ros2 topic echo` and `ros2 service call` for debugging
- Leverage logging with appropriate levels (debug, info, warn, error)
- Use visualization tools like RViz for spatial data

## Summary

This chapter covered the fundamental concepts of ROS 2, including its architecture and purpose. You now understand the basic building blocks of ROS 2 systems:
- Nodes as the fundamental computational units
- Topics for asynchronous publish/subscribe communication
- Services for synchronous request/reply communication
- Advanced concepts like lifecycle nodes, actions, and QoS settings

These concepts form the foundation for all subsequent topics in this module. The examples provided demonstrate practical implementations of these concepts using Python and rclpy, which will be explored in more detail in the next chapter.

## Practical Exercises

1. Create a simple ROS 2 node that prints "Hello, ROS 2!" to the console.
2. Modify the publisher example to publish a counter value every second.
3. Run a publisher and subscriber node simultaneously and observe the message exchange.
4. Implement a lifecycle node that transitions through different states.
5. Create a simple action server and client to understand goal-based communication.