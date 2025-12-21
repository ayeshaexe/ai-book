# Data Model: ROS 2 Module 1 - The Robotic Nervous System

## Entity: ROS 2 Node
- **Description**: A process that performs computation in the ROS 2 system, communicating with other nodes through topics and services
- **Attributes**:
  - node_name: string (unique identifier for the node)
  - node_namespace: string (optional namespace for organization)
  - publishers: list of Publisher objects
  - subscribers: list of Subscriber objects
  - services: list of Service objects
  - clients: list of Client objects
- **Relationships**: Can communicate with other nodes via topics and services
- **Validation**: Must have a unique name within its namespace

## Entity: ROS 2 Topic
- **Description**: A communication channel where nodes can publish or subscribe to messages
- **Attributes**:
  - topic_name: string (name of the topic)
  - message_type: string (type definition of messages on this topic)
  - publishers_count: integer (number of publishers)
  - subscribers_count: integer (number of subscribers)
- **Relationships**: Connected to Publishers and Subscribers
- **Validation**: Message type must be well-defined and compatible across publishers/subscribers

## Entity: ROS 2 Service
- **Description**: A synchronous communication pattern where nodes send requests and receive responses
- **Attributes**:
  - service_name: string (name of the service)
  - service_type: string (type definition of request/response)
  - available: boolean (whether service is currently available)
- **Relationships**: Connected to Service Servers and Service Clients
- **Validation**: Service type must be well-defined with both request and response message types

## Entity: rclpy Client Library
- **Description**: Python client library for ROS 2 that enables Python programs to interact with ROS 2 systems
- **Attributes**:
  - version: string (version of rclpy)
  - supported_nodes: list of Node objects that can be created
  - message_types: list of supported message types
- **Relationships**: Used by Python agents to interact with ROS 2 system
- **Validation**: Must be compatible with the ROS 2 distribution being used

## Entity: URDF Model
- **Description**: Unified Robot Description Format that describes robot structure including links, joints, and inertial properties
- **Attributes**:
  - robot_name: string (name of the robot)
  - links: list of Link objects
  - joints: list of Joint objects
  - materials: list of Material objects
  - gazebo_extensions: list of Gazebo-specific extensions
- **Relationships**: Represents the physical structure of a humanoid robot
- **Validation**: Must be well-formed XML with proper ROS 2 compatibility

## Entity: Humanoid Robot Structure
- **Description**: A robot with a body structure similar to humans, typically with a head, torso, two arms, and two legs
- **Attributes**:
  - body_parts: list of Link objects representing body parts
  - joint_connections: list of Joint objects connecting body parts
  - degrees_of_freedom: integer (total number of controllable joints)
  - kinematic_chain: list of joint connections forming chains
- **Relationships**: Implemented through URDF Model entities
- **Validation**: Must follow humanoid anatomical structure principles

## State Transitions

### Node State Transitions
- **Inactive** → **Active**: When node is initialized and starts communication
- **Active** → **Inactive**: When node is shut down gracefully
- **Active** → **Error**: When node encounters a critical error
- **Error** → **Inactive**: When error is resolved or node is reset

### Communication State Transitions
- **Disconnected** → **Connected**: When publisher/subscriber/service connection is established
- **Connected** → **Disconnected**: When connection is lost or terminated
- **Ready** → **Processing**: When message/service request is being handled
- **Processing** → **Ready**: When message/service request is completed