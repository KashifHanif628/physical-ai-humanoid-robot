"""
LangChain Integration for VLA Systems

This script demonstrates how to integrate LangChain with ROS 2
for Vision-Language-Action robotic systems.
"""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from langchain_openai import ChatOpenAI
from langchain.prompts import PromptTemplate, ChatPromptTemplate
from langchain_core.messages import SystemMessage, HumanMessage
from langchain.chains import LLMChain
from langchain.memory import ConversationBufferMemory
from langchain.agents import initialize_agent, Tool, AgentType
import json
import re


class LangChainVLANode:
    """
    A ROS node that integrates LangChain with robotic systems
    """

    def __init__(self):
        rospy.init_node('langchain_vla_node')

        # Initialize LLM
        self.llm = ChatOpenAI(model="gpt-3.5-turbo", temperature=0.1)

        # Initialize memory for conversation context
        self.memory = ConversationBufferMemory(memory_key="chat_history", return_messages=True)

        # Publishers and subscribers
        self.response_pub = rospy.Publisher('/langchain_response', String, queue_size=10)
        self.status_pub = rospy.Publisher('/langchain_status', String, queue_size=10)
        self.command_sub = rospy.Subscriber('/natural_language_command', String, self.command_callback)

        # Action clients
        self.move_base_client = SimpleActionClient('move_base', MoveBaseAction)

        # Create tools for the agent
        self.tools = [
            Tool(
                name="navigation_tool",
                func=self.navigate_to_location,
                description="Useful for navigating to specific locations. Input should be a location name."
            ),
            Tool(
                name="query_environment",
                func=self.query_environment,
                description="Useful for getting information about the current environment."
            ),
            Tool(
                name="execute_action_sequence",
                func=self.execute_action_sequence,
                description="Useful for executing a sequence of robotic actions. Input should be a JSON string with action sequence."
            )
        ]

        # Initialize the agent
        self.agent = initialize_agent(
            tools=self.tools,
            llm=self.llm,
            agent=AgentType.CONVERSATIONAL_REACT_DESCRIPTION,
            verbose=True,
            memory=self.memory
        )

        # Create specific chains for different tasks
        self.planning_chain = self.create_planning_chain()
        self.intent_chain = self.create_intent_chain()

        rospy.loginfo('LangChain VLA Node initialized')

    def command_callback(self, msg):
        """
        Callback for processing natural language commands

        Args:
            msg: String message containing the natural language command
        """
        command = msg.data
        rospy.loginfo(f'Received command: {command}')

        # Publish processing status
        status_msg = String()
        status_msg.data = f"Processing: {command}"
        self.status_pub.publish(status_msg)

        try:
            # Process command with agent
            response = self.agent.run(input=command)

            # Publish response
            response_msg = String()
            response_msg.data = str(response)
            self.response_pub.publish(response_msg)

            rospy.loginfo(f'Agent response: {response}')

        except Exception as e:
            rospy.logerr(f'Error processing command: {e}')
            self.publish_status(f'Error: {str(e)}')

    def create_planning_chain(self):
        """
        Create a chain specifically for planning tasks

        Returns:
            LLMChain for planning tasks
        """
        planning_prompt = ChatPromptTemplate.from_messages([
            SystemMessage(content="""You are a helpful assistant for a Vision-Language-Action robotic system.
            Convert natural language commands into structured action plans for a humanoid robot.
            Respond with a JSON list of actions with the format:
            [
                {{
                    "action": "action_name",
                    "parameters": {{"param1": "value1", "param2": "value2"}},
                    "description": "Brief description"
                }}
            ]
            """),
            HumanMessage(content="{input}")
        ])

        return planning_prompt | self.llm

    def create_intent_chain(self):
        """
        Create a chain specifically for intent classification

        Returns:
            LLMChain for intent classification
        """
        intent_prompt = ChatPromptTemplate.from_messages([
            SystemMessage(content="""Classify the intent of this command for a robotic system.
            Possible intents: navigation, manipulation, cleaning, scanning, reporting, unknown"""),
            HumanMessage(content="{input}")
        ])

        return intent_prompt | self.llm

    def navigate_to_location(self, location):
        """
        Navigate to a specified location

        Args:
            location: Location name as string

        Returns:
            Result of navigation attempt
        """
        rospy.loginfo(f'Navigating to: {location}')

        # Define location coordinates
        location_coordinates = {
            'kitchen': (1.0, 0.0, 0.0),
            'living room': (2.0, 1.0, 1.57),
            'bedroom': (-1.0, 2.0, 3.14),
            'office': (0.0, -1.0, -1.57),
            'dining room': (1.5, 1.5, 0.78)
        }

        if location in location_coordinates:
            x, y, theta = location_coordinates[location]
            success = self.send_navigation_goal(x, y, theta)
            if success:
                return f'Successfully navigated to {location}'
            else:
                return f'Failed to navigate to {location}'
        else:
            return f'Unknown location: {location}. Available locations: {list(location_coordinates.keys())}'

    def query_environment(self, query):
        """
        Query the environment for information

        Args:
            query: Information to query

        Returns:
            Environmental information
        """
        # In a real implementation, this would interface with sensors and maps
        # For now, return a placeholder response
        return f"Environmental query '{query}' processed. Current environment: indoor, room temperature 22°C, no obstacles detected in immediate vicinity."

    def execute_action_sequence(self, action_sequence_json):
        """
        Execute a sequence of actions

        Args:
            action_sequence_json: JSON string with action sequence

        Returns:
            Result of action execution
        """
        try:
            actions = json.loads(action_sequence_json)
            rospy.loginfo(f'Executing action sequence: {actions}')

            for action in actions:
                action_type = action.get('action', 'unknown')
                parameters = action.get('parameters', {})
                description = action.get('description', '')

                rospy.loginfo(f'Executing: {action_type} - {description}')

                # Execute based on action type
                if action_type == 'navigate_to':
                    location = parameters.get('location', 'unknown')
                    self.navigate_to_location(location)
                elif action_type == 'scan':
                    # Placeholder for scanning
                    rospy.loginfo('Scanning environment...')
                elif action_type == 'clean':
                    # Placeholder for cleaning
                    rospy.loginfo('Cleaning area...')
                else:
                    rospy.logwarn(f'Unknown action type: {action_type}')

            return f'Successfully executed {len(actions)} actions'
        except json.JSONDecodeError:
            return 'Invalid JSON in action sequence'
        except Exception as e:
            return f'Error executing action sequence: {str(e)}'

    def send_navigation_goal(self, x, y, theta):
        """
        Send navigation goal to move_base action server

        Args:
            x, y, theta: Goal coordinates

        Returns:
            Boolean indicating success or failure
        """
        # Wait for the action server to be available
        if not self.move_base_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr('Navigation server not available')
            return False

        # Create a goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Set the position
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = theta
        goal.target_pose.pose.orientation.w = 1.0

        # Send the goal
        self.move_base_client.send_goal(goal)

        # Wait for result
        finished_within_time = self.move_base_client.wait_for_result(rospy.Duration(30.0))

        if not finished_within_time:
            rospy.logerr('Navigation goal did not finish within time limit')
            return False

        state = self.move_base_client.get_state()
        success = state == 3  # GoalStatus.SUCCEEDED

        if success:
            rospy.loginfo('Navigation goal succeeded')
        else:
            rospy.logerr(f'Navigation goal failed with state: {state}')

        return success

    def plan_complex_task(self, command):
        """
        Plan a complex task using the planning chain

        Args:
            command: Natural language command

        Returns:
            Structured action plan
        """
        try:
            response = self.planning_chain.invoke({"input": command})

            # Extract JSON from response
            content = response.content if hasattr(response, 'content') else str(response)
            json_match = re.search(r'\[.*\]', content, re.DOTALL)

            if json_match:
                json_str = json_match.group(0)
                plan = json.loads(json_str)
                return plan
            else:
                rospy.logwarn('No JSON found in planning response')
                return self.create_simple_plan(command)
        except Exception as e:
            rospy.logerr(f'Error in complex planning: {e}')
            return self.create_simple_plan(command)

    def create_simple_plan(self, command):
        """
        Create a simple plan if complex planning fails

        Args:
            command: Natural language command

        Returns:
            Simple action plan
        """
        command_lower = command.lower()

        if 'clean' in command_lower:
            return [
                {"action": "navigate_to", "parameters": {"location": "room_center"}, "description": "Go to center of room"},
                {"action": "scan", "parameters": {}, "description": "Scan for cleaning targets"},
                {"action": "clean", "parameters": {}, "description": "Clean the area"}
            ]
        elif 'go to' in command_lower or 'navigate to' in command_lower:
            # Extract location
            import re
            match = re.search(r'(?:go to|navigate to|move to)\s+(.+)', command_lower)
            location = match.group(1).strip() if match else "unknown"
            return [
                {"action": "navigate_to", "parameters": {"location": location}, "description": f"Go to {location}"}
            ]
        else:
            return [
                {"action": "query_environment", "parameters": {"query": command}, "description": "Query environment about command"}
            ]

    def publish_status(self, status):
        """
        Publish status message

        Args:
            status: Status string to publish
        """
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)


def main():
    """
    Main function to demonstrate LangChain integration
    """
    node = LangChainVLANode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down LangChain VLA node")


if __name__ == '__main__':
    main()