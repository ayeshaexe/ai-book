---
title: Voice-to-Action with Whisper
sidebar_position: 1
---

# Voice-to-Action with Whisper

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand Whisper integration for voice recognition in robotics
- Implement voice command recognition pipeline
- Map voice commands to ROS 2 actions
- Handle edge cases in voice recognition

## Prerequisites

- Intermediate robotics knowledge
- Python programming skills
- Basic understanding of ROS 2
- Familiarity with AI/ML concepts

**Estimated Reading Time**: 45 minutes

## Introduction

Voice-to-Action represents a critical component in the Vision-Language-Action (VLA) framework for humanoid robotics. This chapter explores how OpenAI Whisper enables natural human-robot interaction by converting spoken commands into executable robotic actions. The integration of voice recognition with robotic systems creates an intuitive interface that allows humans to communicate with robots using natural language.

Voice-to-Action systems bridge the gap between human communication and robotic execution. In the context of humanoid robotics, this involves several key components:
- Voice recognition and transcription
- Natural language understanding
- Action mapping and execution
- Feedback and confirmation mechanisms

## Common Terminology and Concepts

Throughout this module, we'll use specific terminology related to Vision-Language-Action systems:

- **Voice Command**: Natural language instruction provided by user through speech
- **Whisper**: OpenAI's automatic speech recognition (ASR) system
- **Transcription**: Conversion of spoken language to text
- **Confidence Score**: Measure of how certain the system is about its transcription
- **ROS 2 Action**: Specific robot command or behavior that executes part of a plan
- **Action Mapping**: Process of converting recognized text commands to ROS 2 actions
- **Humanoid Robot**: Two-legged robot platform capable of navigation, manipulation, and interaction
- **Voice-to-Action Pipeline**: Complete workflow from speech input to robotic action execution
- **Noise Reduction**: Techniques to improve voice recognition accuracy in noisy environments
- **Command Context**: Information about the environment and robot state during command processing

## Whisper Integration Workflow

The Whisper integration workflow in robotics applications involves several distinct stages:

1. **Audio Capture**: Collecting voice input from microphones or audio sensors
2. **Preprocessing**: Cleaning and preparing audio for recognition
3. **Transcription**: Converting speech to text using Whisper models
4. **Intent Recognition**: Understanding the purpose of the command
5. **Action Mapping**: Translating the command to specific ROS 2 actions
6. **Execution**: Carrying out the robotic action
7. **Feedback**: Confirming successful completion to the user

### Whisper Model Selection and Tradeoffs

When implementing Whisper for robotics applications, several model options are available, each with specific tradeoffs:

- **Whisper Tiny**: Fastest processing, lowest accuracy, smallest footprint
- **Whisper Base**: Balanced speed and accuracy
- **Whisper Small**: Good accuracy, moderate resource requirements
- **Whisper Medium**: High accuracy, significant computational requirements
- **Whisper Large**: Highest accuracy, most resource-intensive

For robotics applications, Whisper Medium is typically recommended as it provides the best balance between accuracy and computational efficiency. The model must be able to handle various accents, background noise, and environmental conditions common in robotic environments.

### Example Whisper Integration Code

```python
import whisper
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class VoiceToActionNode:
    def __init__(self):
        # Initialize Whisper model
        self.model = whisper.load_model("medium")

        # Initialize ROS node
        rospy.init_node('voice_to_action')

        # Publishers for different action types
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.action_pub = rospy.Publisher('/robot_actions', String, queue_size=10)

        # Audio input setup would go here
        self.audio_queue = []

    def transcribe_audio(self, audio_data):
        """
        Transcribe audio data using Whisper
        """
        # Convert audio to appropriate format for Whisper
        result = self.model.transcribe(audio_data)
        return result["text"], result["confidence"]

    def parse_command(self, text):
        """
        Parse natural language command and extract intent
        """
        text = text.lower().strip()

        # Simple keyword-based parsing (in practice, use more sophisticated NLU)
        if "move forward" in text or "go forward" in text:
            return "move_forward", {}
        elif "move backward" in text or "go back" in text:
            return "move_backward", {}
        elif "turn left" in text or "rotate left" in text:
            return "turn_left", {}
        elif "turn right" in text or "rotate right" in text:
            return "turn_right", {}
        elif "stop" in text:
            return "stop", {}
        elif "pick up" in text or "grasp" in text:
            # Extract object to pick up
            obj = self.extract_object(text)
            return "pick_up", {"object": obj}
        else:
            return "unknown", {"command": text}

    def extract_object(self, text):
        """
        Extract object name from command (simplified)
        """
        # This is a simplified approach - in practice, use NER or similar
        words = text.split()
        for i, word in enumerate(words):
            if word in ["the", "a", "an"]:
                if i + 1 < len(words):
                    return words[i + 1]
        return "object"

    def execute_action(self, action_type, params):
        """
        Execute the appropriate ROS 2 action based on command
        """
        if action_type == "move_forward":
            self.move_forward()
        elif action_type == "move_backward":
            self.move_backward()
        elif action_type == "turn_left":
            self.turn_left()
        elif action_type == "turn_right":
            self.turn_right()
        elif action_type == "stop":
            self.stop_robot()
        elif action_type == "pick_up":
            self.pick_up_object(params.get("object", "object"))
        else:
            rospy.logwarn(f"Unknown action type: {action_type}")

    def move_forward(self):
        """Move robot forward"""
        cmd = Twist()
        cmd.linear.x = 0.5  # Forward velocity
        self.cmd_vel_pub.publish(cmd)
        rospy.loginfo("Moving forward")

    def move_backward(self):
        """Move robot backward"""
        cmd = Twist()
        cmd.linear.x = -0.5  # Backward velocity
        self.cmd_vel_pub.publish(cmd)
        rospy.loginfo("Moving backward")

    def turn_left(self):
        """Turn robot left"""
        cmd = Twist()
        cmd.angular.z = 0.5  # Left turn
        self.cmd_vel_pub.publish(cmd)
        rospy.loginfo("Turning left")

    def turn_right(self):
        """Turn robot right"""
        cmd = Twist()
        cmd.angular.z = -0.5  # Right turn
        self.cmd_vel_pub.publish(cmd)
        rospy.loginfo("Turning right")

    def stop_robot(self):
        """Stop robot movement"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        rospy.loginfo("Robot stopped")

    def pick_up_object(self, obj_name):
        """Attempt to pick up an object"""
        action_msg = String()
        action_msg.data = f"pick_up:{obj_name}"
        self.action_pub.publish(action_msg)
        rospy.loginfo(f"Attempting to pick up {obj_name}")

    def process_voice_command(self, audio_data):
        """
        Complete pipeline: audio -> text -> command -> action
        """
        try:
            # Step 1: Transcribe audio
            text, confidence = self.transcribe_audio(audio_data)
            rospy.loginfo(f"Transcribed: '{text}' with confidence: {confidence:.2f}")

            # Step 2: Check confidence threshold
            if confidence < 0.7:
                rospy.logwarn("Low confidence transcription, ignoring command")
                return False

            # Step 3: Parse command
            action_type, params = self.parse_command(text)

            # Step 4: Execute action
            self.execute_action(action_type, params)

            return True

        except Exception as e:
            rospy.logerr(f"Error processing voice command: {str(e)}")
            return False

# Example usage
if __name__ == '__main__':
    try:
        voice_node = VoiceToActionNode()
        # In a real implementation, you would have a loop that continuously
        # listens for audio input and processes commands
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
```

## Audio Processing and Noise Reduction

Audio processing is a critical component of voice-to-action systems, especially in robotics environments where background noise is common. Effective noise reduction ensures that Whisper can accurately transcribe commands even in challenging acoustic conditions.

### Preprocessing Techniques

Before audio is passed to Whisper, several preprocessing steps can improve recognition accuracy:

- **Noise Reduction**: Apply filters to remove background noise
- **Audio Normalization**: Adjust volume levels to consistent ranges
- **Silence Detection**: Identify and remove long periods of silence
- **Audio Format Conversion**: Ensure audio is in the correct format for Whisper

### Conceptual Diagram: Whisper Integration

```
┌─────────────────────────────────────────────────────────────┐
│                Whisper Integration Pipeline                 │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐     │
│  │   Audio     │───▶│  Whisper    │───▶│  Command    │     │
│  │   Capture   │    │ Transcribe  │    │  Parsing    │     │
│  │             │    │             │    │             │     │
│  └─────────────┘    └─────────────┘    └─────────────┘     │
│         │                   │                   │          │
│         ▼                   ▼                   ▼          │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐     │
│  │ Preprocess  │    │ Confidence  │    │ Intent      │     │
│  │  Audio      │    │  Check      │    │  Recognition│     │
│  │             │    │             │    │             │     │
│  └─────────────┘    └─────────────┘    └─────────────┘     │
│         │                   │                   │          │
│         └───────────────────┼───────────────────┘          │
│                             ▼                              │
│  ┌─────────────────────────────────────────────────────┐   │
│  │              Action Mapping & Execution             │   │
│  │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐   │   │
│  │  │  Map to     │ │ Validate    │ │ Execute     │   │   │
│  │  │ ROS Action  │ │  Action     │ │  Action     │   │   │
│  │  └─────────────┘ └─────────────┘ └─────────────┘   │   │
│  └─────────────────────────────────────────────────────┘   │
│                             │                              │
│  ┌─────────────────────────────────────────────────────┐   │
│  │                 Feedback & Logging                  │   │
│  │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐   │   │
│  │  │  Success    │ │   Error     │ │  User       │   │   │
│  │  │  Report     │ │   Report    │ │  Feedback   │   │   │
│  │  └─────────────┘ └─────────────┘ └─────────────┘   │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

## Voice Command Recognition Setup

Setting up a robust voice command recognition system involves several key components:

### Microphone Configuration

- **Quality**: Use high-quality microphones with good sensitivity
- **Positioning**: Position microphones to capture clear voice input
- **Array Setup**: Consider microphone arrays for better noise cancellation
- **Sampling Rate**: Use appropriate sampling rates (typically 16kHz for Whisper)

### Environmental Considerations

- **Acoustic Environment**: Account for room acoustics and background noise
- **Distance**: Optimize for the expected distance between user and robot
- **Directionality**: Consider directional microphones for focused capture
- **Echo Cancellation**: Implement echo cancellation for better clarity

## ROS 2 Action Mapping Concepts

The mapping between recognized voice commands and ROS 2 actions involves several important concepts:

### Action Types

- **Navigation Actions**: Movement commands (move forward, turn, navigate to location)
- **Manipulation Actions**: Object interaction commands (pick up, place, grasp)
- **Perception Actions**: Sensing commands (look, detect, identify)
- **State Actions**: Robot state changes (stop, pause, resume)

### Command Validation

Before executing any action, the system should validate:

- **Safety**: Ensure the action won't cause harm
- **Feasibility**: Check if the robot is capable of the requested action
- **Context**: Verify the action makes sense in the current context
- **Parameters**: Validate any parameters included in the command

## Summary and Key Takeaways

Voice-to-Action systems form the foundation of natural human-robot interaction in the VLA framework. Key takeaways include:

- Whisper provides high-quality speech recognition capabilities for robotics applications
- Proper audio preprocessing is essential for robust performance in real environments
- The pipeline from voice input to robotic action requires careful design and validation
- Command parsing and action mapping must be reliable and safe
- Confidence scoring helps filter out unreliable transcriptions
- The system should provide clear feedback to users about command processing

The successful implementation of voice-to-action capabilities enables intuitive human-robot interaction, making robots more accessible and easier to control for non-expert users.

## Key Terms Glossary

- **Voice Command**: Natural language instruction provided by user through speech
- **Whisper**: OpenAI's automatic speech recognition (ASR) system
- **Transcription**: Conversion of spoken language to text
- **Confidence Score**: Measure of how certain the system is about its transcription
- **ROS 2 Action**: Specific robot command or behavior that executes part of a plan
- **Action Mapping**: Process of converting recognized text commands to ROS 2 actions
- **Humanoid Robot**: Two-legged robot platform capable of navigation, manipulation, and interaction
- **Voice-to-Action Pipeline**: Complete workflow from speech input to robotic action execution
- **Noise Reduction**: Techniques to improve voice recognition accuracy in noisy environments
- **Command Context**: Information about the environment and robot state during command processing

## Exercises and Practical Activities

1. **Whisper Setup**: Install and test Whisper with sample audio files
2. **Command Mapping**: Create a simple mapping system for basic robot commands
3. **Confidence Testing**: Experiment with different confidence thresholds
4. **Noise Simulation**: Test voice recognition with varying levels of background noise

## Further Reading and Resources

- [OpenAI Whisper Documentation](https://github.com/openai/whisper)
- [ROS 2 Navigation Documentation](https://navigation.ros.org/)
- [Speech Recognition in Robotics](https://ieeexplore.ieee.org/document/9149031)
- [Human-Robot Interaction Guidelines](https://humanrobotinteraction.org/)

## See Also

- [Chapter 2: Cognitive Planning with LLMs](./chap-2-cognitive-planning.md) - Learn about LLM-based planning for robotic actions
- [Chapter 3: Capstone - Autonomous Humanoid](./chap-3-capstone.md) - Explore end-to-end VLA integration

## Navigation Links

[Next: Chapter 2 - Cognitive Planning with LLMs](./chap-2-cognitive-planning.md)