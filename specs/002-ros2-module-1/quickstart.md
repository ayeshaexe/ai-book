# Quickstart Guide: ROS 2 Module 1 - The Robotic Nervous System

## Prerequisites

Before starting this module, ensure you have:
- Basic Python knowledge (functions, classes, modules)
- Understanding of fundamental programming concepts
- Access to a system with ROS 2 installed (Humble Hawksbill or later recommended)

## Environment Setup

### 1. Install ROS 2
```bash
# Follow the official ROS 2 installation guide for your OS
# For Ubuntu: http://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
# For Windows: http://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html
# For macOS: http://docs.ros.org/en/humble/Installation/macOS-Install-Binary.html
```

### 2. Verify Installation
```bash
# Check ROS 2 installation
ros2 --version

# Source ROS 2 environment
source /opt/ros/humble/setup.bash  # For Ubuntu/Humble
```

### 3. Create a Workspace
```bash
# Create workspace directory
mkdir -p ~/ros2_module_ws/src
cd ~/ros2_module_ws

# Build the workspace
colcon build
source install/setup.bash
```

## Chapter 1: ROS 2 Fundamentals

### Understanding Nodes, Topics, and Services

1. **Run a simple publisher node**:
```bash
ros2 run demo_nodes_cpp talker
```

2. **In another terminal, run a subscriber**:
```bash
ros2 run demo_nodes_py listener
```

3. **Observe the communication between nodes**:
```bash
# List active topics
ros2 topic list

# Echo messages on a topic
ros2 topic echo /chatter std_msgs/msg/String
```

## Chapter 2: Python Agents and ROS Control

### Using rclpy to Create Python Nodes

1. **Create a simple Python publisher**:
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

2. **Run your Python node**:
```bash
python3 your_publisher_script.py
```

## Chapter 3: Humanoid Robot Modeling with URDF

### Understanding URDF Structure

1. **View a sample URDF file**:
```bash
# Create a simple URDF file to understand the structure
gedit simple_robot.urdf
```

2. **Visualize URDF with RViz**:
```bash
# Launch RViz with URDF visualization
ros2 launch urdf_tutorial display.launch.py model:=path/to/your/robot.urdf
```

## Verification Steps

After completing each chapter, verify your understanding:

### Chapter 1 Verification
- [ ] Can explain the difference between nodes, topics, and services
- [ ] Can identify message flow in a simple ROS 2 system
- [ ] Can run basic ROS 2 commands (ros2 topic list, ros2 node list)

### Chapter 2 Verification
- [ ] Can create a Python node using rclpy
- [ ] Can implement publisher and subscriber patterns in Python
- [ ] Can exchange messages between Python nodes

### Chapter 3 Verification
- [ ] Can read and interpret a URDF file
- [ ] Can identify links, joints, and overall structure in a URDF model
- [ ] Can visualize a simple robot model in RViz

## Common Issues and Solutions

### Issue: ROS 2 commands not found
**Solution**: Ensure you've sourced the ROS 2 environment:
```bash
source /opt/ros/humble/setup.bash
```

### Issue: Nodes can't communicate
**Solution**: Check that nodes are on the same ROS domain ID:
```bash
echo $ROS_DOMAIN_ID  # Should be the same for communicating nodes
```

### Issue: Python import errors
**Solution**: Ensure rclpy is installed:
```bash
pip3 install ros2-rclpy
```

## Next Steps

After completing this module, you should:
1. Have a solid understanding of ROS 2 fundamentals
2. Be able to create Python agents that interact with ROS 2 systems
3. Understand how to interpret URDF models for humanoid robots
4. Be ready for more advanced robotics concepts in subsequent modules