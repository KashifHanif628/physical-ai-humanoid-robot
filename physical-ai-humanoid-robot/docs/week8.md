---
title: "Human-Robot Interaction & Conversational AI"
sidebar_label: "Week 8: Human-Robot Interaction"
description: "Learn about human-robot interaction design and conversational AI integration"
keywords: ["hri", "conversational ai", "gpt", "interaction", "social robotics", "dialogue"]
---

# Week 8: Human-Robot Interaction & Conversational AI

## Introduction

This final week focuses on human-robot interaction (HRI) design and conversational AI integration, completing the Physical AI system with natural communication capabilities. You'll learn to design intuitive interfaces, integrate GPT models for conversational AI, and implement effective human-robot collaboration patterns. This completes the full Physical AI system that bridges AI from digital systems to physical humanoid robots with natural communication.

## Human-Robot Interaction Fundamentals

Human-robot interaction is a multidisciplinary field that combines robotics, psychology, human factors, and computer science to create effective and intuitive interfaces between humans and robots.

### HRI Design Principles

1. **Transparency**: Robots should clearly communicate their intentions and state
2. **Predictability**: Robot behavior should be consistent and understandable
3. **Trust**: Design interactions that build and maintain human trust
4. **Social Cues**: Use appropriate social signals and behaviors
5. **Safety**: Prioritize human safety in all interactions
6. **Accessibility**: Ensure interfaces are usable by diverse populations

### Interaction Modalities

Humanoid robots can interact through multiple modalities:

- **Verbal**: Spoken language and voice synthesis
- **Gestural**: Body language, pointing, and expressive movements
- **Facial**: Eye contact, facial expressions, and gaze
- **Proxemic**: Spatial relationships and personal space
- **Tactile**: Physical interaction and haptic feedback

## Conversational AI Integration

Conversational AI enables natural language interaction between humans and robots. For Physical AI systems, this involves integrating large language models (LLMs) with physical robot capabilities.

### GPT Integration Architecture

```python
import openai
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time
import json
import asyncio
from typing import Dict, List, Optional

class ConversationalAINode(Node):
    def __init__(self):
        super().__init__('conversational_ai_node')

        # Initialize OpenAI client
        self.openai_client = openai.OpenAI(api_key=self.get_parameter_or('openai_api_key', ''))

        # GPT model configuration
        self.gpt_model = self.get_parameter_or('gpt_model', 'gpt-3.5-turbo')
        self.temperature = self.get_parameter_or('temperature', 0.7)
        self.max_tokens = self.get_parameter_or('max_tokens', 500)

        # Publishers and subscribers
        self.response_publisher = self.create_publisher(String, '/hri/response', 10)
        self.command_publisher = self.create_publisher(String, '/robot/commands', 10)
        self.status_publisher = self.create_publisher(String, '/hri/status', 10)

        self.user_input_subscriber = self.create_subscription(
            String, '/hri/user_input', self.user_input_callback, 10
        )
        self.robot_state_subscriber = self.create_subscription(
            String, '/robot/state', self.robot_state_callback, 10
        )
        self.environment_subscriber = self.create_subscription(
            String, '/environment/state', self.environment_callback, 10
        )

        # Context management
        self.conversation_history = []
        self.robot_state = {}
        self.environment_state = {}

        # Command mapping
        self.command_keywords = {
            'move': ['go to', 'move to', 'navigate to', 'walk to'],
            'grasp': ['pick up', 'grasp', 'take', 'get'],
            'place': ['put', 'place', 'set down'],
            'inspect': ['look at', 'examine', 'check'],
            'follow': ['follow me', 'come with me'],
            'stop': ['stop', 'halt', 'wait'],
            'home': ['go home', 'return home', 'charge']
        }

        self.get_logger().info('Conversational AI Node initialized')

    def user_input_callback(self, msg):
        """
        Process user input through GPT and generate appropriate response
        """
        user_input = msg.data
        self.get_logger().info(f'Received user input: {user_input}')

        # Update status
        status_msg = String()
        status_msg.data = f"Processing: {user_input}"
        self.status_publisher.publish(status_msg)

        try:
            # Generate response using GPT
            response = self.generate_gpt_response(user_input)

            # Publish response
            response_msg = String()
            response_msg.data = response
            self.response_publisher.publish(response_msg)

            # Check if response contains commands for the robot
            robot_commands = self.extract_robot_commands(response)
            if robot_commands:
                for cmd in robot_commands:
                    cmd_msg = String()
                    cmd_msg.data = cmd
                    self.command_publisher.publish(cmd_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing user input: {e}')
            error_msg = String()
            error_msg.data = f"Sorry, I encountered an error: {str(e)}"
            self.response_publisher.publish(error_msg)

    def generate_gpt_response(self, user_input):
        """
        Generate response using GPT model with context
        """
        # Prepare system context
        system_context = self.get_system_context()

        # Prepare conversation history
        messages = [
            {"role": "system", "content": system_context},
        ]

        # Add conversation history (limit to last 10 exchanges)
        recent_history = self.conversation_history[-10:] if len(self.conversation_history) > 10 else self.conversation_history
        messages.extend(recent_history)

        # Add current user input
        messages.append({"role": "user", "content": user_input})

        # Call GPT API
        response = self.openai_client.chat.completions.create(
            model=self.gpt_model,
            messages=messages,
            temperature=self.temperature,
            max_tokens=self.max_tokens
        )

        # Extract response content
        gpt_response = response.choices[0].message.content

        # Update conversation history
        self.conversation_history.append({"role": "user", "content": user_input})
        self.conversation_history.append({"role": "assistant", "content": gpt_response})

        return gpt_response

    def get_system_context(self):
        """
        Get system context for GPT including robot capabilities and environment
        """
        context = f"""
        You are a humanoid robot with the following capabilities:
        - Navigation: Can move to specified locations
        - Manipulation: Can grasp and place objects
        - Perception: Can recognize objects and people
        - Communication: Can engage in natural conversation

        Current robot state: {json.dumps(self.robot_state)}
        Current environment state: {json.dumps(self.environment_state)}

        Respond naturally to human requests. If the request involves physical actions,
        format your response to include specific commands in JSON format like:
        {{'commands': ['NAVIGATE_TO_LOCATION: kitchen', 'GRASP_OBJECT: red cup']}}

        Keep responses concise but helpful. Acknowledge your physical limitations.
        """

        return context

    def extract_robot_commands(self, gpt_response):
        """
        Extract robot commands from GPT response
        """
        commands = []

        # Look for JSON commands in the response
        try:
            # Find JSON blocks in response
            import re
            json_blocks = re.findall(r'\{[^{}]*\}', gpt_response)

            for block in json_blocks:
                try:
                    data = json.loads(block)
                    if 'commands' in data:
                        commands.extend(data['commands'])
                except json.JSONDecodeError:
                    continue
        except Exception as e:
            self.get_logger().warn(f'Error extracting commands from response: {e}')

        # Also look for keyword-based commands
        lower_response = gpt_response.lower()

        for cmd_type, keywords in self.command_keywords.items():
            for keyword in keywords:
                if keyword in lower_response:
                    # Extract target if available
                    target = self.extract_target_from_sentence(lower_response, keyword)

                    if cmd_type == 'move':
                        commands.append(f"NAVIGATE_TO: {target if target else 'location_not_specified'}")
                    elif cmd_type == 'grasp':
                        commands.append(f"GRASP: {target if target else 'object_not_specified'}")
                    elif cmd_type == 'place':
                        commands.append(f"PLACE_AT: {target if target else 'location_not_specified'}")
                    elif cmd_type == 'follow':
                        commands.append("FOLLOW_HUMAN: ACTIVE")
                    elif cmd_type == 'stop':
                        commands.append("STOP_MOVEMENT: IMMEDIATE")
                    elif cmd_type == 'home':
                        commands.append("RETURN_HOME: START")

        return commands

    def extract_target_from_sentence(self, sentence, command_keyword):
        """
        Extract target object/location from sentence containing command keyword
        """
        # Find the command keyword and extract what comes after
        idx = sentence.find(command_keyword)
        if idx != -1:
            remaining = sentence[idx + len(command_keyword):].strip()

            # Look for common indicators of targets
            words = remaining.split()
            if words:
                # Take first few words as potential target
                target_words = []
                for word in words[:3]:  # Look at first 3 words
                    # Remove punctuation and common articles
                    clean_word = word.strip('.,!?;:')
                    if clean_word.lower() not in ['the', 'a', 'an', 'to', 'at', 'in', 'on']:
                        target_words.append(clean_word)

                if target_words:
                    return ' '.join(target_words[:2])  # Take first 2 meaningful words

        return None

    def robot_state_callback(self, msg):
        """
        Update robot state from robot state publisher
        """
        try:
            self.robot_state = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Invalid JSON in robot state message')

    def environment_callback(self, msg):
        """
        Update environment state from environment publisher
        """
        try:
            self.environment_state = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Invalid JSON in environment state message')
```

### Advanced Conversational Patterns

```python
class AdvancedConversationHandler:
    """
    Advanced conversation handler with dialogue management and context awareness
    """

    def __init__(self):
        self.dialogue_state = {
            'current_task': None,
            'task_step': 0,
            'entities': {},
            'user_preferences': {},
            'conversation_context': {}
        }

        self.task_flows = {
            'navigation': [
                'confirm_destination',
                'check_route_feasibility',
                'request_permission',
                'execute_navigation',
                'confirm_arrival'
            ],
            'manipulation': [
                'identify_object',
                'verify_grasp_feasibility',
                'request_permission',
                'execute_grasp',
                'confirm_completion'
            ],
            'inspection': [
                'identify_target',
                'position_for_inspection',
                'analyze_target',
                'report_results'
            ]
        }

    def handle_navigation_request(self, user_input, robot_state, environment_state):
        """
        Handle navigation requests with proper task flow
        """
        # Extract destination from user input
        destination = self.extract_destination(user_input)

        if not destination:
            # Ask for clarification
            return {
                'response': 'Where would you like me to go?',
                'awaiting_input': 'destination',
                'task_context': 'navigation'
            }

        # Check if destination is feasible
        is_feasible, reason = self.check_navigation_feasibility(destination, robot_state, environment_state)

        if not is_feasible:
            return {
                'response': f"I cannot navigate to {destination} because {reason}",
                'task_completed': False
            }

        # Confirm navigation request
        self.dialogue_state['current_task'] = 'navigation'
        self.dialogue_state['task_step'] = 0
        self.dialogue_state['entities']['destination'] = destination

        return {
            'response': f"I can navigate to {destination}. Should I proceed?",
            'awaiting_confirmation': True,
            'task_context': 'navigation'
        }

    def handle_manipulation_request(self, user_input, robot_state, environment_state):
        """
        Handle manipulation requests with proper task flow
        """
        # Extract object and action from user input
        action, object_name = self.extract_manipulation_request(user_input)

        if not object_name:
            # Ask for clarification
            return {
                'response': 'What object would you like me to manipulate?',
                'awaiting_input': 'object',
                'task_context': 'manipulation'
            }

        # Check if object is reachable and graspable
        object_info = self.locate_object(object_name, environment_state)
        if not object_info:
            return {
                'response': f"I cannot find the {object_name} nearby.",
                'task_completed': False
            }

        grasp_feasible, grasp_type = self.check_grasp_feasibility(object_info, robot_state)

        if not grasp_feasible:
            return {
                'response': f"I cannot grasp the {object_name} with my current configuration.",
                'task_completed': False
            }

        # Confirm manipulation request
        self.dialogue_state['current_task'] = 'manipulation'
        self.dialogue_state['task_step'] = 0
        self.dialogue_state['entities']['object'] = object_name
        self.dialogue_state['entities']['grasp_type'] = grasp_type

        action_desc = {
            'grasp': f'grasp the {object_name}',
            'place': f'place the {object_name}',
            'move': f'move the {object_name}'
        }.get(action, f'{action} the {object_name}')

        return {
            'response': f"I can {action_desc}. Should I proceed?",
            'awaiting_confirmation': True,
            'task_context': 'manipulation'
        }

    def process_user_confirmation(self, user_input):
        """
        Process user confirmation for pending tasks
        """
        if self.dialogue_state['current_task'] is None:
            return {
                'response': "There's no pending task to confirm.",
                'task_completed': False
            }

        user_input_lower = user_input.lower()

        if any(word in user_input_lower for word in ['yes', 'ok', 'sure', 'proceed', 'go ahead']):
            # Execute the confirmed task
            task_result = self.execute_pending_task()

            if task_result['success']:
                return {
                    'response': task_result['response'],
                    'task_completed': True,
                    'robot_command': task_result.get('command')
                }
            else:
                return {
                    'response': f"Task failed: {task_result['error']}",
                    'task_completed': False
                }
        elif any(word in user_input_lower for word in ['no', 'cancel', 'stop', 'never mind']):
            # Cancel the task
            self.reset_dialogue_state()
            return {
                'response': "Task cancelled.",
                'task_completed': True
            }
        else:
            # Unclear response
            return {
                'response': "I didn't understand. Please say yes to proceed or no to cancel.",
                'awaiting_confirmation': True
            }

    def execute_pending_task(self):
        """
        Execute the currently pending task
        """
        if self.dialogue_state['current_task'] == 'navigation':
            return self.execute_navigation_task()
        elif self.dialogue_state['current_task'] == 'manipulation':
            return self.execute_manipulation_task()
        elif self.dialogue_state['current_task'] == 'inspection':
            return self.execute_inspection_task()
        else:
            return {
                'success': False,
                'error': f"Unknown task type: {self.dialogue_state['current_task']}"
            }

    def execute_navigation_task(self):
        """
        Execute navigation task with safety checks
        """
        destination = self.dialogue_state['entities'].get('destination')

        # Check safety conditions
        if not self.check_navigation_safety(destination):
            return {
                'success': False,
                'error': 'Navigation safety check failed'
            }

        # Generate navigation command
        command = f"NAVIGATE_TO: {destination}"

        # Reset task state
        self.reset_dialogue_state()

        return {
            'success': True,
            'response': f"Okay, navigating to {destination}.",
            'command': command
        }

    def execute_manipulation_task(self):
        """
        Execute manipulation task with grasp planning
        """
        object_name = self.dialogue_state['entities'].get('object')
        grasp_type = self.dialogue_state['entities'].get('grasp_type')

        # Check safety conditions
        if not self.check_manipulation_safety(object_name):
            return {
                'success': False,
                'error': 'Manipulation safety check failed'
            }

        # Generate manipulation command
        command = f"GRASP_OBJECT: {object_name} WITH {grasp_type}"

        # Reset task state
        self.reset_dialogue_state()

        return {
            'success': True,
            'response': f"Okay, grasping the {object_name}.",
            'command': command
        }

    def reset_dialogue_state(self):
        """
        Reset dialogue state to default
        """
        self.dialogue_state = {
            'current_task': None,
            'task_step': 0,
            'entities': {},
            'user_preferences': {},
            'conversation_context': {}
        }
```

## Social Robotics and Expressive Behavior

Humanoid robots need expressive capabilities to enhance human-robot interaction through social cues and behaviors.

### Expressive Movement System

```python
import math
import numpy as np

class ExpressiveBehaviorController:
    """
    Controller for expressive behaviors and social cues
    """

    def __init__(self):
        self.behavior_patterns = {
            'attention': self.attention_movement,
            'confirmation': self.confirmation_movement,
            'acknowledgment': self.acknowledgment_movement,
            'thinking': self.thinking_movement,
            'greeting': self.greeting_movement,
            'farewell': self.farewell_movement
        }

    def attention_movement(self):
        """
        Express attention through head and eye movements
        """
        # Saccadic eye movements to look at person
        # Slight head tilt to show engagement
        return {
            'head_pitch': 0.1,  # Slightly tilted
            'head_yaw': 0.0,    # Looking forward
            'eye_movement': self.generate_saccadic_motion(),
            'posture': 'attentive'
        }

    def confirmation_movement(self):
        """
        Express confirmation through nodding
        """
        return {
            'head_pitch': 0.2,  # Nod
            'gesture': 'thumbs_up',
            'posture': 'affirmative'
        }

    def acknowledgment_movement(self):
        """
        Express acknowledgment through subtle movements
        """
        return {
            'head_nod': 0.1,
            'eyebrow_raise': 0.1,
            'posture': 'responsive'
        }

    def thinking_movement(self):
        """
        Express thinking through subtle movements
        """
        return {
            'head_tilt': 0.15,
            'eye_gaze': 'up_and_away',  # Looking up as if thinking
            'posture': 'contemplative'
        }

    def greeting_movement(self):
        """
        Express greeting through appropriate gesture
        """
        return {
            'head_nod': 0.2,
            'wave_arm': 0.5,  # Gentle wave
            'posture': 'welcoming'
        }

    def farewell_movement(self):
        """
        Express farewell through appropriate gesture
        """
        return {
            'head_nod': 0.15,
            'wave_arm': 0.8,  # Wave goodbye
            'posture': 'respectful'
        }

    def generate_saccadic_motion(self):
        """
        Generate realistic eye saccadic movements
        """
        # Saccadic movements are quick, simultaneous movements of both eyes
        # in the same direction
        movements = []
        for _ in range(3):  # 3 quick movements
            x_offset = np.random.uniform(-0.1, 0.1)
            y_offset = np.random.uniform(-0.05, 0.05)
            duration = np.random.uniform(0.05, 0.1)  # 50-100ms

            movements.append({
                'x': x_offset,
                'y': y_offset,
                'duration': duration
            })

        return movements

    def execute_behavior(self, behavior_type, intensity=1.0):
        """
        Execute specified behavior with given intensity
        """
        if behavior_type in self.behavior_patterns:
            base_movement = self.behavior_patterns[behavior_type]()

            # Scale movements by intensity
            scaled_movement = self.scale_movement_by_intensity(base_movement, intensity)

            return scaled_movement
        else:
            return None

    def scale_movement_by_intensity(self, movement, intensity):
        """
        Scale movement parameters by intensity factor
        """
        scaled = {}
        for key, value in movement.items():
            if isinstance(value, (int, float)):
                scaled[key] = value * intensity
            else:
                scaled[key] = value

        return scaled
```

## Safety and Ethics in HRI

Safety and ethical considerations are paramount in human-robot interaction, especially for humanoid robots operating in human environments.

### Safety Framework

```python
class HRISafetyFramework:
    """
    Safety framework for human-robot interaction
    """

    def __init__(self):
        # Safety zones around robot
        self.safety_zones = {
            'critical': 0.3,    # Immediate danger zone (touching distance)
            'caution': 0.8,     # Zone where robot should be extra careful
            'awareness': 1.5    # Zone where robot should be aware of humans
        }

        # Safety priorities
        self.safety_priorities = [
            'human_safety',
            'robot_safety',
            'task_completion',
            'social_norms'
        ]

    def check_interaction_safety(self, human_positions, robot_state):
        """
        Check if planned interaction is safe given human positions
        """
        safety_status = {
            'safe': True,
            'violations': [],
            'risk_level': 'low'
        }

        for human_pos in human_positions:
            distance_to_human = self.calculate_distance(robot_state['position'], human_pos)

            if distance_to_human < self.safety_zones['critical']:
                safety_status['safe'] = False
                safety_status['violations'].append(f'HUMAN_TOO_CLOSE: Distance {distance_to_human:.2f}m < {self.safety_zones["critical"]}m')
                safety_status['risk_level'] = 'high'
            elif distance_to_human < self.safety_zones['caution']:
                safety_status['violations'].append(f'CLOSE_PROXIMITY: Distance {distance_to_human:.2f}m < {self.safety_zones["caution"]}m')
                if safety_status['risk_level'] == 'low':
                    safety_status['risk_level'] = 'medium'

        return safety_status

    def enforce_safety_constraints(self, planned_action, human_positions, robot_state):
        """
        Enforce safety constraints on planned action
        """
        safety_check = self.check_interaction_safety(human_positions, robot_state)

        if not safety_check['safe']:
            # Modify action to be safer
            if 'NAVIGATE' in planned_action:
                # Adjust navigation to maintain safe distance
                return self.adjust_navigation_for_safety(planned_action, human_positions, robot_state)
            elif 'GRASP' in planned_action:
                # Check if manipulation is safe
                return self.check_manipulation_safety(planned_action, human_positions, robot_state)
            else:
                # Stop the action
                return 'SAFETY_STOP: Unsafe action prevented'

        return planned_action

    def calculate_distance(self, pos1, pos2):
        """
        Calculate Euclidean distance between two positions
        """
        return math.sqrt(sum([(a - b) ** 2 for a, b in zip(pos1, pos2)]))

    def adjust_navigation_for_safety(self, navigation_command, human_positions, robot_state):
        """
        Adjust navigation command to maintain safe distances from humans
        """
        # Extract target from command
        if ':' in navigation_command:
            target_str = navigation_command.split(':')[1].strip()
            # This is a simplified example - in reality, you'd parse the target location
            target_pos = self.get_position_for_location(target_str)

            # Calculate safe path that avoids humans
            safe_path = self.calculate_safe_path_to_target(robot_state['position'], target_pos, human_positions)

            if safe_path:
                return f"NAVIGATE_VIA: {safe_path}"
            else:
                return "NAVIGATION_CANCELLED: No safe path available"

        return navigation_command

    def get_position_for_location(self, location_name):
        """
        Get position coordinates for a named location
        """
        # This would be populated with actual location coordinates
        location_map = {
            'kitchen': [2.0, 1.0, 0.0],
            'living_room': [0.0, 0.0, 0.0],
            'bedroom': [-1.0, 2.0, 0.0],
            'office': [1.5, -1.0, 0.0]
        }

        return location_map.get(location_name, [0.0, 0.0, 0.0])

    def calculate_safe_path_to_target(self, start_pos, target_pos, human_positions):
        """
        Calculate a safe path that avoids humans
        """
        # Simplified path planning - in reality, this would use more sophisticated algorithms
        # like RRT*, A*, or D* Lite

        # Check if direct path is safe
        direct_path_clear = True
        for human_pos in human_positions:
            if self.path_intersects_human(start_pos, target_pos, human_pos, self.safety_zones['caution']):
                direct_path_clear = False
                break

        if direct_path_clear:
            return [target_pos]  # Direct path is safe

        # For this simplified version, return None if direct path isn't safe
        # A real implementation would calculate a detour
        return None

    def path_intersects_human(self, start, end, human_pos, safety_radius):
        """
        Check if path from start to end intersects with human safety zone
        """
        # Simplified: check if closest point on line segment is within safety radius
        # This is a basic implementation - real systems would use more accurate methods
        start_vec = np.array(start[:2])  # Only consider x,y for 2D path
        end_vec = np.array(end[:2])
        human_vec = np.array(human_pos[:2])

        # Calculate distance from line segment to human position
        line_vec = end_vec - start_vec
        human_to_start = human_vec - start_vec

        # Project human position onto line
        line_len_sq = np.dot(line_vec, line_vec)
        if line_len_sq == 0:
            dist_sq = np.dot(human_to_start, human_to_start)
        else:
            t = max(0, min(1, np.dot(human_to_start, line_vec) / line_len_sq))
            projection = start_vec + t * line_vec
            dist_sq = np.dot(human_vec - projection, human_vec - projection)

        return dist_sq <= (safety_radius ** 2)
```

## ROS 2 Integration for HRI

Here's how to integrate the HRI system with ROS 2:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, Vector3
from sensor_msgs.msg import Image, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from std_srvs.srv import SetBool
import json
import threading
import queue

class HumanoidHRIController(Node):
    def __init__(self):
        super().__init__('humanoid_hri_controller')

        # Initialize components
        self.conversational_ai = ConversationalAINode()
        self.behavior_controller = ExpressiveBehaviorController()
        self.safety_framework = HRISafetyFramework()

        # Publishers
        self.response_publisher = self.create_publisher(String, '/hri/response', 10)
        self.behavior_command_publisher = self.create_publisher(String, '/hri/behavior_commands', 10)
        self.safety_status_publisher = self.create_publisher(String, '/hri/safety_status', 10)
        self.visual_feedback_publisher = self.create_publisher(MarkerArray, '/hri/visual_feedback', 10)
        self.robot_command_publisher = self.create_publisher(String, '/robot/commands', 10)

        # Subscribers
        self.user_input_subscriber = self.create_subscription(
            String, '/hri/user_input', self.user_input_callback, 10
        )
        self.human_detection_subscriber = self.create_subscription(
            String, '/perception/human_detection', self.human_detection_callback, 10
        )
        self.robot_state_subscriber = self.create_subscription(
            String, '/robot/state', self.robot_state_callback, 10
        )
        self.environment_subscriber = self.create_subscription(
            String, '/environment/state', self.environment_callback, 10
        )

        # Services
        self.interaction_permission_service = self.create_service(
            SetBool, '/hri/request_interaction_permission', self.interaction_permission_callback
        )

        # Timers
        self.hri_timer = self.create_timer(0.1, self.hri_control_loop)  # 10 Hz
        self.safety_timer = self.create_timer(0.05, self.safety_monitoring_loop)  # 20 Hz

        # State variables
        self.human_positions = []
        self.robot_state = {}
        self.environment_state = {}
        self.conversation_active = False
        self.safe_to_interact = True

        # Interaction queues
        self.user_input_queue = queue.Queue()
        self.response_queue = queue.Queue()

        self.get_logger().info('Humanoid HRI Controller initialized')

    def user_input_callback(self, msg):
        """
        Handle user input from microphone or text interface
        """
        self.get_logger().info(f'Received user input: {msg.data}')

        # Add to processing queue
        self.user_input_queue.put(msg.data)

    def human_detection_callback(self, msg):
        """
        Update detected human positions
        """
        try:
            human_data = json.loads(msg.data)
            self.human_positions = human_data.get('humans', [])
        except json.JSONDecodeError:
            self.get_logger().warn('Invalid JSON in human detection message')

    def robot_state_callback(self, msg):
        """
        Update robot state
        """
        try:
            self.robot_state = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Invalid JSON in robot state message')

    def environment_callback(self, msg):
        """
        Update environment state
        """
        try:
            self.environment_state = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Invalid JSON in environment state message')

    def hri_control_loop(self):
        """
        Main HRI control loop
        """
        # Process user inputs
        while not self.user_input_queue.empty():
            try:
                user_input = self.user_input_queue.get_nowait()
                self.process_user_input(user_input)
            except queue.Empty:
                break

        # Process responses
        while not self.response_queue.empty():
            try:
                response = self.response_queue.get_nowait()
                self.publish_response(response)
            except queue.Empty:
                break

        # Check safety constraints
        safety_status = self.safety_framework.check_interaction_safety(
            self.human_positions, self.robot_state
        )

        if not safety_status['safe']:
            self.safe_to_interact = False
            self.get_logger().warn(f'Safety violation: {safety_status["violations"]}')
        else:
            self.safe_to_interact = True

    def safety_monitoring_loop(self):
        """
        Safety monitoring running at higher frequency
        """
        if not self.safe_to_interact:
            # Emergency stop if unsafe
            stop_msg = String()
            stop_msg.data = "EMERGENCY_STOP: Safety violation detected"
            self.robot_command_publisher.publish(stop_msg)

    def process_user_input(self, user_input):
        """
        Process user input through conversational AI system
        """
        if not self.safe_to_interact:
            response = "I cannot interact right now for safety reasons."
            self.response_queue.put(response)
            return

        try:
            # Generate response using GPT
            gpt_response = self.conversational_ai.generate_gpt_response(user_input)

            # Extract robot commands from response
            robot_commands = self.conversational_ai.extract_robot_commands(gpt_response)

            # Publish robot commands if any
            if robot_commands:
                for cmd in robot_commands:
                    # Check safety constraints before executing
                    safe_command = self.safety_framework.enforce_safety_constraints(
                        cmd, self.human_positions, self.robot_state
                    )

                    if safe_command != 'SAFETY_STOP: Unsafe action prevented':
                        cmd_msg = String()
                        cmd_msg.data = safe_command
                        self.robot_command_publisher.publish(cmd_msg)

            # Publish response
            self.response_queue.put(gpt_response)

            # Trigger appropriate expressive behavior
            self.trigger_expressive_behavior(gpt_response)

        except Exception as e:
            self.get_logger().error(f'Error processing user input: {e}')
            error_response = "Sorry, I encountered an error processing your request."
            self.response_queue.put(error_response)

    def trigger_expressive_behavior(self, response):
        """
        Trigger appropriate expressive behavior based on response content
        """
        if any(word in response.lower() for word in ['yes', 'okay', 'sure', 'confirmed']):
            behavior = self.behavior_controller.execute_behavior('confirmation')
        elif any(word in response.lower() for word in ['thinking', 'let me', 'consider', 'hmm']):
            behavior = self.behavior_controller.execute_behavior('thinking')
        elif any(greeting in response.lower() for greeting in ['hello', 'hi', 'greetings', 'good morning']):
            behavior = self.behavior_controller.execute_behavior('greeting')
        else:
            behavior = self.behavior_controller.execute_behavior('attention')

        if behavior:
            behavior_msg = String()
            behavior_msg.data = json.dumps(behavior)
            self.behavior_command_publisher.publish(behavior_msg)

    def interaction_permission_callback(self, request, response):
        """
        Handle interaction permission requests
        """
        if self.safe_to_interact:
            response.success = True
            response.message = "Interaction permitted"
        else:
            response.success = False
            response.message = "Interaction denied due to safety concerns"

        return response

    def publish_response(self, response):
        """
        Publish response to appropriate topics
        """
        response_msg = String()
        response_msg.data = response
        self.response_publisher.publish(response_msg)

        # Also publish to text-to-speech if available
        tts_msg = String()
        tts_msg.data = response
        # Assuming there's a TTS publisher available
        # self.tts_publisher.publish(tts_msg)

def main(args=None):
    rclpy.init(args=args)

    node = HumanoidHRIController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch File for HRI System

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    openai_api_key = LaunchConfiguration('openai_api_key')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    declare_openai_api_key_arg = DeclareLaunchArgument(
        'openai_api_key',
        default_value='',
        description='OpenAI API key for GPT integration'
    )

    # Conversational AI node
    conversational_ai_node = Node(
        package='physical_ai_humanoid_hri',
        executable='conversational_ai_node',
        name='conversational_ai',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'openai_api_key': openai_api_key},
            {'gpt_model': 'gpt-3.5-turbo'},
            {'temperature': 0.7},
            {'max_tokens': 500}
        ],
        remappings=[
            ('/hri/user_input', '/hri/text_input'),
            ('/hri/response', '/hri/text_response'),
            ('/robot/state', '/robot_state'),
            ('/environment/state', '/environment_state'),
        ],
        output='screen'
    )

    # HRI controller node
    hri_controller = Node(
        package='physical_ai_humanoid_hri',
        executable='hri_controller_node',
        name='hri_controller',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Voice interface node
    voice_interface = Node(
        package='physical_ai_humanoid_hri',
        executable='voice_interface_node',
        name='voice_interface',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Text-to-speech node
    text_to_speech = Node(
        package='physical_ai_humanoid_hri',
        executable='tts_node',
        name='text_to_speech',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_openai_api_key_arg)

    # Add nodes
    ld.add_action(conversational_ai_node)
    ld.add_action(hri_controller)
    ld.add_action(voice_interface)
    ld.add_action(text_to_speech)

    return ld
```

## Best Practices for HRI Systems

### 1. Ethical Considerations
- Design for transparency and accountability
- Respect user privacy and data protection
- Ensure equitable access and avoid bias
- Consider psychological impact on users

### 2. Safety First
- Implement multiple safety layers and fail-safes
- Maintain safe distances during interaction
- Monitor for unexpected human behavior
- Provide clear emergency stop mechanisms

### 3. Natural Interaction
- Use appropriate social cues and behaviors
- Maintain natural conversation flow
- Provide clear feedback on robot state
- Adapt interaction style to user preferences

### 4. Robustness
- Handle ambiguous or incorrect user input gracefully
- Maintain conversation context across turns
- Recover from system failures appropriately
- Provide fallback communication methods

## Learning Objectives

By the end of Week 8, you should be able to:
1. Design intuitive human-robot interaction interfaces
2. Integrate GPT models for conversational AI capabilities
3. Implement expressive behaviors for social robotics
4. Apply safety and ethical principles to HRI systems
5. Create complete Physical AI systems with natural communication

## Exercises

### Exercise 1: Basic Conversational Interface (Beginner)
- **Time**: 60 minutes
- **Objective**: Implement a basic conversational interface with GPT integration
- **Steps**: Create a simple system that accepts text input and generates GPT responses
- **Expected Outcome**: Working conversational system that can engage in basic dialogue

### Exercise 2: Context-Aware Conversation (Intermediate)
- **Time**: 90 minutes
- **Objective**: Implement context-aware conversation with robot state integration
- **Steps**: Create system that incorporates robot and environment state into conversations
- **Expected Outcome**: Conversational system that understands robot capabilities and context

### Exercise 3: Safe HRI System (Advanced)
- **Time**: 120 minutes
- **Objective**: Implement complete HRI system with safety monitoring and expressive behaviors
- **Steps**: Create system with safety constraints, expressive behaviors, and natural dialogue
- **Expected Outcome**: Complete HRI system that operates safely with natural interaction patterns

## Summary

Week 8 completed the Physical AI course by covering human-robot interaction and conversational AI integration. You learned to design intuitive interfaces, integrate GPT models for natural communication, implement expressive behaviors, and apply safety and ethical principles to HRI systems. This completes the comprehensive Physical AI & Humanoid Robotics course, providing students with knowledge spanning from Physical AI fundamentals through advanced HRI systems. The course now provides a complete foundation for developing humanoid robots that can perceive, plan, act, and interact naturally with humans in real-world environments.