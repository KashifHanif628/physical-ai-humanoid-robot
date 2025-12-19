"""
LLM Planning Node for VLA Systems

This script demonstrates how to use LLMs for cognitive planning
in Vision-Language-Action robotic systems.
"""

import rospy
import json
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from langchain_openai import ChatOpenAI
from langchain.prompts import PromptTemplate
import re


class LLMPlanningNode:
    """
    A ROS node that uses LLMs for cognitive planning and task decomposition
    """

    def __init__(self):
        rospy.init_node('llm_planning_node')

        # Initialize LLM
        self.llm = ChatOpenAI(model="gpt-3.5-turbo")

        # Publishers and subscribers
        self.plan_pub = rospy.Publisher('/planned_actions', String, queue_size=10)
        self.status_pub = rospy.Publisher('/planning_status', String, queue_size=10)
        self.command_sub = rospy.Subscriber('/natural_language_command', String, self.command_callback)

        # Action clients
        self.move_base_client = SimpleActionClient('move_base', MoveBaseAction)

        # Planning prompt template
        self.planning_prompt = PromptTemplate.from_template(
            """Convert this natural language command into a structured action plan.
            Command: {command}
            Environment: {environment}
            Robot capabilities: {capabilities}
            Current robot state: {state}

            Return a JSON list of actions with the following structure:
            [
                {{
                    "action": "action_name",
                    "parameters": {{"param1": "value1", "param2": "value2"}},
                    "description": "Brief description",
                    "dependencies": ["action_id1", "action_id2"]  // Optional dependencies
                }}
            ]

            Available actions: navigate_to, pick_up, place, clean, scan, wait, report
            Ensure the plan is safe and executable by a ROS 2 robot."""
        )

        rospy.loginfo('LLM Planning Node initialized')

    def command_callback(self, msg):
        """
        Callback for processing natural language commands

        Args:
            msg: String message containing the natural language command
        """
        command = msg.data
        rospy.loginfo(f'Received natural language command: {command}')

        # Publish planning status
        status_msg = String()
        status_msg.data = f"Planning for: {command}"
        self.status_pub.publish(status_msg)

        try:
            # Generate plan using LLM
            plan = self.generate_plan(command)

            if plan:
                # Publish the plan
                plan_msg = String()
                plan_msg.data = json.dumps(plan)
                self.plan_pub.publish(plan_msg)

                # Execute the plan
                success = self.execute_plan(plan)
                rospy.loginfo(f"Plan execution {'succeeded' if success else 'failed'}")
            else:
                rospy.logerr('Failed to generate plan')
                self.publish_status('Planning failed')

        except Exception as e:
            rospy.logerr(f'Error in planning: {e}')
            self.publish_status(f'Planning error: {str(e)}')

    def generate_plan(self, command):
        """
        Generate a structured action plan using LLM

        Args:
            command: Natural language command string

        Returns:
            List of action dictionaries or None if generation failed
        """
        try:
            # Prepare context for the LLM
            environment = self.get_environment_context()
            capabilities = self.get_robot_capabilities()
            state = self.get_robot_state()

            # Create the chain
            chain = self.planning_prompt | self.llm

            # Generate the response
            response = chain.invoke({
                "command": command,
                "environment": environment,
                "capabilities": capabilities,
                "state": state
            })

            # Extract JSON from response
            plan_json = self.extract_json_from_response(response.content)

            if plan_json:
                plan = json.loads(plan_json)
                rospy.loginfo(f'Generated plan: {plan}')
                return plan
            else:
                rospy.logwarn('No JSON found in LLM response')
                return self.create_fallback_plan(command)

        except Exception as e:
            rospy.logerr(f'Error generating plan: {e}')
            return self.create_fallback_plan(command)

    def extract_json_from_response(self, response_text):
        """
        Extract JSON from LLM response text

        Args:
            response_text: Text response from LLM

        Returns:
            JSON string or None if not found
        """
        # Look for JSON array in the response
        start_idx = response_text.find('[')
        end_idx = response_text.rfind(']')

        if start_idx != -1 and end_idx != -1 and end_idx > start_idx:
            json_str = response_text[start_idx:end_idx + 1]
            return json_str

        return None

    def create_fallback_plan(self, command):
        """
        Create a fallback plan if LLM generation fails

        Args:
            command: The original command

        Returns:
            List of action dictionaries
        """
        command_lower = command.lower()

        # Simple rule-based fallback
        if 'clean' in command_lower:
            return [
                {
                    "action": "navigate_to",
                    "parameters": {"location": "room_center"},
                    "description": "Go to center of room for cleaning",
                    "dependencies": []
                },
                {
                    "action": "clean",
                    "parameters": {"area": "current"},
                    "description": "Clean the current area",
                    "dependencies": ["navigate_to"]
                }
            ]
        elif 'go to' in command_lower or 'navigate to' in command_lower:
            # Extract location using regex
            match = re.search(r'(?:go to|navigate to|move to)\s+(.+)', command_lower)
            location = match.group(1).strip() if match else "unknown"
            return [
                {
                    "action": "navigate_to",
                    "parameters": {"location": location},
                    "description": f"Navigate to {location}",
                    "dependencies": []
                }
            ]
        elif 'pick' in command_lower or 'get' in command_lower:
            return [
                {
                    "action": "navigate_to",
                    "parameters": {"location": "object_location"},
                    "description": "Go to object location",
                    "dependencies": []
                },
                {
                    "action": "pick_up",
                    "parameters": {"object": "specified_object"},
                    "description": "Pick up the object",
                    "dependencies": ["navigate_to"]
                }
            ]

        return [
            {
                "action": "unknown_command",
                "parameters": {"command": command},
                "description": "Unable to parse command",
                "dependencies": []
            }
        ]

    def get_environment_context(self):
        """
        Get current environment context

        Returns:
            String describing the environment
        """
        # In a real implementation, this would query environment sensors
        return "indoor environment with furniture, obstacles, and navigation markers"

    def get_robot_capabilities(self):
        """
        Get robot capabilities context

        Returns:
            String describing robot capabilities
        """
        return "navigation, manipulation, perception, voice interaction, basic cleaning tools"

    def get_robot_state(self):
        """
        Get current robot state

        Returns:
            String describing robot state
        """
        # In a real implementation, this would query robot state
        return "charged, idle, at home position, no current tasks"

    def execute_plan(self, plan):
        """
        Execute a structured action plan

        Args:
            plan: List of action dictionaries

        Returns:
            Boolean indicating success or failure
        """
        rospy.loginfo(f'Executing plan with {len(plan)} actions')

        for action in plan:
            success = self.execute_single_action(action)
            if not success:
                rospy.logerr(f'Action failed: {action}')
                return False

        rospy.loginfo('Plan executed successfully')
        self.publish_status('Plan completed successfully')
        return True

    def execute_single_action(self, action):
        """
        Execute a single action based on its type

        Args:
            action: Action dictionary

        Returns:
            Boolean indicating success or failure
        """
        action_type = action.get('action', 'unknown')
        parameters = action.get('parameters', {})

        rospy.loginfo(f'Executing action: {action_type} with parameters: {parameters}')

        if action_type == 'navigate_to':
            return self.execute_navigation_action(parameters)
        elif action_type == 'pick_up':
            return self.execute_manipulation_action(parameters)
        elif action_type == 'clean':
            return self.execute_cleaning_action(parameters)
        elif action_type == 'scan':
            return self.execute_scan_action(parameters)
        elif action_type == 'wait':
            return self.execute_wait_action(parameters)
        elif action_type == 'report':
            return self.execute_report_action(parameters)
        elif action_type == 'unknown_command':
            rospy.logwarn('Received unknown command action')
            return False
        else:
            rospy.logwarn(f'Unknown action type: {action_type}')
            return False

    def execute_navigation_action(self, parameters):
        """
        Execute navigation action

        Args:
            parameters: Action parameters

        Returns:
            Boolean indicating success or failure
        """
        location = parameters.get('location', 'unknown')

        # In a real implementation, this would convert location name to coordinates
        # For now, use a simple coordinate mapping
        location_coordinates = {
            'kitchen': (1.0, 0.0, 0.0),
            'living room': (2.0, 1.0, 1.57),
            'bedroom': (-1.0, 2.0, 3.14),
            'room_center': (0.0, 0.0, 0.0)
        }

        if location in location_coordinates:
            x, y, theta = location_coordinates[location]
            return self.send_navigation_goal(x, y, theta)
        else:
            rospy.logwarn(f'Unknown location: {location}')
            return False

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

    def execute_manipulation_action(self, parameters):
        """
        Execute manipulation action

        Args:
            parameters: Action parameters

        Returns:
            Boolean indicating success or failure
        """
        # Placeholder for manipulation actions
        # In a real implementation, this would interface with manipulation controllers
        rospy.loginfo('Executing manipulation action')
        return True

    def execute_cleaning_action(self, parameters):
        """
        Execute cleaning action

        Args:
            parameters: Action parameters

        Returns:
            Boolean indicating success or failure
        """
        # Placeholder for cleaning actions
        # In a real implementation, this would interface with cleaning tools
        rospy.loginfo('Executing cleaning action')
        return True

    def execute_scan_action(self, parameters):
        """
        Execute scan action

        Args:
            parameters: Action parameters

        Returns:
            Boolean indicating success or failure
        """
        # Placeholder for scanning actions
        # In a real implementation, this would interface with sensors
        rospy.loginfo('Executing scan action')
        return True

    def execute_wait_action(self, parameters):
        """
        Execute wait action

        Args:
            parameters: Action parameters

        Returns:
            Boolean indicating success or failure
        """
        duration = parameters.get('duration', 1.0)
        rospy.loginfo(f'Waiting for {duration} seconds')
        rospy.sleep(duration)
        return True

    def execute_report_action(self, parameters):
        """
        Execute report action

        Args:
            parameters: Action parameters

        Returns:
            Boolean indicating success or failure
        """
        message = parameters.get('message', 'No message provided')
        rospy.loginfo(f'Report: {message}')
        return True

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
    Main function to demonstrate LLM planning
    """
    planner = LLMPlanningNode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down LLM planning node")


if __name__ == '__main__':
    main()