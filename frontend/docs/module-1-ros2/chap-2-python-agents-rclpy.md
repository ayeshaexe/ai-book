---
title: Python Agents and ROS Control
sidebar_label: Chapter 2 - Python Agents and ROS Control
---

# Chapter 2: Python Agents and ROS Control

## Introduction

This chapter focuses on connecting Python agents to ROS controllers using rclpy, the Python client library for ROS 2. Python is a popular choice for robotics applications due to its simplicity and extensive library support, making it ideal for rapid prototyping and development. Understanding how to create Python agents that communicate with ROS systems is essential for implementing robotic applications.

## Concepts

### Python in Robotics

Python is widely used in robotics for:
- Prototyping and testing algorithms
- Developing user interfaces
- Data processing and analysis
- Machine learning and AI applications
- Scripting and automation tasks

### rclpy Overview

rclpy is the Python client library for ROS 2. It provides:
- APIs for creating ROS 2 nodes
- Publishers and subscribers for topic-based communication
- Clients and services for request-response communication
- Parameter management
- Lifecycle management

rclpy serves as the bridge between Python applications and the ROS 2 middleware, allowing Python developers to leverage the powerful features of ROS 2 while working in a familiar language.

### Agent-Controller Communication Patterns

In robotics, agents often need to communicate with controllers to perform tasks. Common patterns include:
- Requesting sensor data from hardware controllers
- Sending commands to actuator controllers
- Receiving feedback about system status
- Coordinating with other agents and controllers

## Examples

### Creating a Python Agent Node

Here's a complete example of a Python agent that connects to ROS controllers:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class RobotAgent(Node):
    def __init__(self):
        super().__init__('robot_agent')

        # Publisher for sending movement commands to the robot controller
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for receiving sensor data from the robot
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        # Timer for periodic agent behavior
        self.timer = self.create_timer(0.1, self.agent_behavior)

        self.get_logger().info('Robot Agent initialized')

    def laser_callback(self, msg):
        """Process laser scan data from the robot's sensors"""
        # Example: Check for obstacles in front of the robot
        if len(msg.ranges) > 0:
            front_distance = msg.ranges[len(msg.ranges) // 2]  # Front is middle of array
            if front_distance < 1.0:  # If obstacle is closer than 1 meter
                self.get_logger().info(f'Obstacle detected at {front_distance:.2f}m')

    def agent_behavior(self):
        """Main agent behavior loop"""
        # Example: Simple movement command
        cmd = Twist()
        cmd.linear.x = 0.2  # Move forward at 0.2 m/s
        cmd.angular.z = 0.0  # No rotation
        self.cmd_vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    agent = RobotAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client Example

Here's an example of a Python agent that uses services to communicate with controllers:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from example_interfaces.srv import AddTwoInts

class ServiceAgent(Node):
    def __init__(self):
        super().__init__('service_agent')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.get_logger().info('Service client initialized')

    def send_request(self, a, b):
        """Send a request to the service and return the future"""
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.cli.call_async(request)
        return future

def main(args=None):
    rclpy.init(args=args)
    agent = ServiceAgent()

    # Send a request
    future = agent.send_request(2, 3)

    # Spin until the future is complete
    while rclpy.ok():
        rclpy.spin_once(agent)
        if future.done():
            try:
                response = future.result()
                agent.get_logger().info(f'Result: {response.sum}')
            except Exception as e:
                agent.get_logger().info(f'Service call failed: {e}')
            break

    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Parameter Management Example

Here's how to use parameters in your Python agent:

```python
import rclpy
from rclpy.node import Node

class ParameterAgent(Node):
    def __init__(self):
        super().__init__('parameter_agent')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'turtlebot')
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('safety_distance', 0.8)

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_speed = self.get_parameter('max_speed').value
        self.safety_distance = self.get_parameter('safety_distance').value

        self.get_logger().info(f'Robot: {self.robot_name}, Max speed: {self.max_speed}, Safety distance: {self.safety_distance}')

def main(args=None):
    rclpy.init(args=args)
    agent = ParameterAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

This chapter explored how to create Python agents that communicate with ROS controllers using rclpy. You learned about:
- Creating Python nodes that can act as agents in a ROS system
- Implementing publishers and subscribers for topic-based communication
- Using services for request-response communication patterns
- Managing parameters in Python agents

These concepts enable you to create sophisticated Python agents that can interact with various ROS controllers and systems. The examples provided demonstrate practical implementations of agent-controller communication patterns that are commonly used in robotics applications.

## Advanced Agent Patterns

### State Machines in Agents

Complex robotic agents often implement state machines to manage different operational modes. A state machine helps organize the agent's behavior based on its current state and environmental conditions. Common states in robotic agents include idle, exploring, following, avoiding obstacles, and returning to base.

Here's an example of a simple state machine implementation in a Python agent:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class StateMachineAgent(Node):
    def __init__(self):
        super().__init__('state_machine_agent')

        # Publisher for sending movement commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for sensor data
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        # Timer for state machine updates
        self.timer = self.create_timer(0.1, self.state_machine)

        # Initialize state
        self.state = 'IDLE'  # IDLE, FORWARD, TURN
        self.obstacle_detected = False

        self.get_logger().info('State Machine Agent initialized')

    def laser_callback(self, msg):
        """Process laser scan data"""
        if len(msg.ranges) > 0:
            front_distance = min(msg.ranges[:10] + msg.ranges[-10:])  # Check front area
            self.obstacle_detected = front_distance < 0.8

    def state_machine(self):
        """State machine implementation"""
        if self.state == 'IDLE':
            if not self.obstacle_detected:
                self.state = 'FORWARD'
        elif self.state == 'FORWARD':
            if self.obstacle_detected:
                self.state = 'TURN'
            else:
                # Move forward
                cmd = Twist()
                cmd.linear.x = 0.2
                self.cmd_vel_publisher.publish(cmd)
        elif self.state == 'TURN':
            if not self.obstacle_detected:
                self.state = 'FORWARD'
            else:
                # Turn in place
                cmd = Twist()
                cmd.angular.z = 0.5
                self.cmd_vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    agent = StateMachineAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Behavior Trees

For more complex robotic behaviors, behavior trees provide a more structured approach than simple state machines. Behavior trees allow for hierarchical composition of behaviors and are particularly useful for complex robotic tasks.

### Agent Coordination

In multi-agent systems, coordination becomes crucial. Python agents can coordinate through:
- Shared topics for broadcasting information
- Services for direct communication
- Parameters for shared configuration
- Actions for complex coordinated tasks

## Integration with ROS 2 Ecosystem

### Using ROS 2 Tools with Python Agents

Python agents can leverage various ROS 2 tools for development and debugging:

- `ros2 run`: To launch your Python agents
- `ros2 topic`: To monitor and debug topic communication
- `ros2 param`: To configure your agents at runtime
- `rqt`: For GUI-based monitoring and interaction
- `RViz`: For visualization of sensor data and robot state

### Launch Files for Python Agents

Launch files provide a convenient way to start multiple nodes with specific configurations:

```xml
<launch>
  <!-- Robot agent -->
  <node pkg="my_robot_pkg" exec="robot_agent" name="navigation_agent">
    <param name="robot_name" value="turtlebot"/>
    <param name="max_speed" value="0.5"/>
    <param name="safety_distance" value="0.8"/>
  </node>

  <!-- Additional nodes -->
  <node pkg="my_robot_pkg" exec="sensor_processor" name="lidar_processor"/>
</launch>
```

## Error Handling and Robustness

### Exception Handling in ROS 2 Nodes

Proper error handling is crucial for robust robotic applications:

```python
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
import traceback

class RobustAgent(Node):
    def __init__(self):
        super().__init__('robust_agent')

        try:
            # Declare and get parameters
            self.declare_parameter('robot_name', 'default_robot')
            self.robot_name = self.get_parameter('robot_name').value
        except ParameterNotDeclaredException:
            self.get_logger().error('Parameter not declared, using default')
            self.robot_name = 'default_robot'

        # Initialize other components safely
        self.initialize_components()

    def initialize_components(self):
        """Safely initialize agent components"""
        try:
            # Component initialization code
            self.get_logger().info(f'Agent {self.robot_name} initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize components: {e}')
            self.get_logger().error(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    agent = RobustAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        agent.get_logger().info('Interrupted by user')
    except Exception as e:
        agent.get_logger().error(f'Unexpected error: {e}')
        agent.get_logger().error(traceback.format_exc())
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Considerations

### Threading and Concurrency

Python agents can benefit from proper threading strategies:

- Use separate threads for I/O operations to avoid blocking the main thread
- Be aware of Python's GIL (Global Interpreter Lock) when designing concurrent operations
- Use ROS 2's built-in multi-threaded executor when needed

### Memory Management

- Properly clean up resources in the node destructor
- Be mindful of memory usage when processing large data streams
- Use generators for processing large datasets when possible

## Testing Python Agents

### Unit Testing

Python agents can be unit tested using standard Python testing frameworks:

```python
import unittest
from unittest.mock import Mock, patch
import rclpy
from your_agent import RobotAgent

class TestRobotAgent(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.agent = RobotAgent()

    def tearDown(self):
        self.agent.destroy_node()
        rclpy.shutdown()

    def test_initialization(self):
        self.assertIsNotNone(self.agent.cmd_vel_publisher)
        self.assertIsNotNone(self.agent.laser_subscriber)

    @patch('geometry_msgs.msg.Twist')
    def test_behavior(self, mock_twist):
        # Test agent behavior logic
        self.agent.agent_behavior()
        # Add assertions based on expected behavior
```

## Summary

This chapter explored how to create Python agents that communicate with ROS controllers using rclpy. You learned about:
- Creating Python nodes that can act as agents in a ROS system
- Implementing publishers and subscribers for topic-based communication
- Using services for request-response communication patterns
- Managing parameters in Python agents
- Advanced patterns like state machines and behavior trees
- Integration with the ROS 2 ecosystem
- Error handling and performance considerations

These concepts enable you to create sophisticated Python agents that can interact with various ROS controllers and systems. The examples provided demonstrate practical implementations of agent-controller communication patterns that are commonly used in robotics applications.

## Practical Exercises

1. Create a Python agent that subscribes to a sensor topic and publishes commands based on the sensor data.
2. Implement a service client in Python that requests data from a robot controller.
3. Create a Python agent with configurable parameters that control its behavior.
4. Implement a state machine in your Python agent to handle different operational modes.
5. Create a launch file that starts your agent with specific parameters.