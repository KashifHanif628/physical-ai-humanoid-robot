"""
Action Graph Generator for VLA Systems

This script demonstrates how to generate ROS action graphs
from natural language commands for Vision-Language-Action robotic systems.
"""

import rospy
import json
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import networkx as nx
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from langchain_openai import ChatOpenAI
from langchain.prompts import PromptTemplate
import threading
import time


class ActionGraphGenerator:
    """
    A class to generate action dependency graphs for robotic execution
    """

    def __init__(self):
        rospy.init_node('action_graph_generator')

        # Initialize LLM
        self.llm = ChatOpenAI(model="gpt-3.5-turbo")

        # Publishers and subscribers
        self.graph_pub = rospy.Publisher('/action_graph', String, queue_size=10)
        self.status_pub = rospy.Publisher('/graph_status', String, queue_size=10)
        self.command_sub = rospy.Subscriber('/natural_language_command', String, self.command_callback)

        # Action clients
        self.move_base_client = SimpleActionClient('move_base', MoveBaseAction)

        # Planning prompt template for graph generation
        self.graph_prompt = PromptTemplate.from_template(
            """Convert this natural language command into a directed action dependency graph.
            Command: {command}
            Environment: {environment}
            Robot capabilities: {capabilities}

            Return a JSON object with the following structure:
            {{
                "nodes": [
                    {{
                        "id": "unique_action_id",
                        "action": "action_name",
                        "parameters": {{"param1": "value1", "param2": "value2"}},
                        "description": "Brief description"
                    }}
                ],
                "edges": [
                    {{
                        "from": "source_action_id",
                        "to": "target_action_id",
                        "relationship": "dependency_type"
                    }}
                ]
            }}

            Ensure the graph represents execution dependencies and can be topologically sorted."""
        )

        rospy.loginfo('Action Graph Generator initialized')

    def command_callback(self, msg):
        """
        Callback for processing natural language commands

        Args:
            msg: String message containing the natural language command
        """
        command = msg.data
        rospy.loginfo(f'Received command for graph generation: {command}')

        # Publish generation status
        status_msg = String()
        status_msg.data = f"Generating graph for: {command}"
        self.status_pub.publish(status_msg)

        try:
            # Generate action graph using LLM
            graph_data = self.generate_action_graph(command)

            if graph_data:
                # Validate the graph structure
                if self.validate_graph_structure(graph_data):
                    # Publish the graph
                    graph_msg = String()
                    graph_msg.data = json.dumps(graph_data)
                    self.graph_pub.publish(graph_msg)

                    # Execute the graph
                    success = self.execute_action_graph(graph_data)
                    rospy.loginfo(f"Graph execution {'succeeded' if success else 'failed'}")
                else:
                    rospy.logerr('Generated graph has structural issues')
                    self.publish_status('Graph validation failed')
            else:
                rospy.logerr('Failed to generate action graph')
                self.publish_status('Graph generation failed')

        except Exception as e:
            rospy.logerr(f'Error in graph generation: {e}')
            self.publish_status(f'Graph generation error: {str(e)}')

    def generate_action_graph(self, command):
        """
        Generate an action dependency graph using LLM

        Args:
            command: Natural language command string

        Returns:
            Dictionary containing nodes and edges or None if generation failed
        """
        try:
            # Prepare context for the LLM
            environment = self.get_environment_context()
            capabilities = self.get_robot_capabilities()

            # Create the chain
            chain = self.graph_prompt | self.llm

            # Generate the response
            response = chain.invoke({
                "command": command,
                "environment": environment,
                "capabilities": capabilities
            })

            # Extract JSON from response
            graph_json = self.extract_json_from_response(response.content)

            if graph_json:
                graph_data = json.loads(graph_json)
                rospy.loginfo(f'Generated action graph: {graph_data}')
                return graph_data
            else:
                rospy.logwarn('No JSON found in LLM response')
                return self.create_fallback_graph(command)

        except Exception as e:
            rospy.logerr(f'Error generating action graph: {e}')
            return self.create_fallback_graph(command)

    def extract_json_from_response(self, response_text):
        """
        Extract JSON from LLM response text

        Args:
            response_text: Text response from LLM

        Returns:
            JSON string or None if not found
        """
        # Look for JSON object in the response
        start_idx = response_text.find('{')
        end_idx = response_text.rfind('}')

        if start_idx != -1 and end_idx != -1 and end_idx > start_idx:
            json_str = response_text[start_idx:end_idx + 1]
            return json_str

        return None

    def create_fallback_graph(self, command):
        """
        Create a fallback action graph if LLM generation fails

        Args:
            command: The original command

        Returns:
            Dictionary containing nodes and edges
        """
        command_lower = command.lower()

        # Simple rule-based fallback graph
        if 'clean' in command_lower:
            return {
                "nodes": [
                    {
                        "id": "navigate_to_room",
                        "action": "navigate_to",
                        "parameters": {"location": "room_center"},
                        "description": "Navigate to the room to clean"
                    },
                    {
                        "id": "scan_area",
                        "action": "scan",
                        "parameters": {},
                        "description": "Scan the area for cleaning targets"
                    },
                    {
                        "id": "clean_area",
                        "action": "clean",
                        "parameters": {"area": "current"},
                        "description": "Clean the scanned area"
                    },
                    {
                        "id": "return_home",
                        "action": "navigate_to",
                        "parameters": {"location": "home_base"},
                        "description": "Return to home position"
                    }
                ],
                "edges": [
                    {
                        "from": "navigate_to_room",
                        "to": "scan_area",
                        "relationship": "precedes"
                    },
                    {
                        "from": "scan_area",
                        "to": "clean_area",
                        "relationship": "precedes"
                    },
                    {
                        "from": "clean_area",
                        "to": "return_home",
                        "relationship": "precedes"
                    }
                ]
            }
        elif 'go to' in command_lower or 'navigate to' in command_lower:
            # Extract location using regex
            import re
            match = re.search(r'(?:go to|navigate to|move to)\s+(.+)', command_lower)
            location = match.group(1).strip() if match else "unknown"

            return {
                "nodes": [
                    {
                        "id": "navigate_to_target",
                        "action": "navigate_to",
                        "parameters": {"location": location},
                        "description": f"Navigate to {location}"
                    }
                ],
                "edges": []
            }
        else:
            return {
                "nodes": [
                    {
                        "id": "unknown_command",
                        "action": "unknown",
                        "parameters": {"command": command},
                        "description": "Unable to parse command"
                    }
                ],
                "edges": []
            }

    def validate_graph_structure(self, graph_data):
        """
        Validate the structure of the action graph

        Args:
            graph_data: Dictionary containing nodes and edges

        Returns:
            Boolean indicating if the graph is valid
        """
        try:
            nodes = graph_data.get('nodes', [])
            edges = graph_data.get('edges', [])

            # Check that nodes and edges are lists
            if not isinstance(nodes, list) or not isinstance(edges, list):
                return False

            # Check that all nodes have required fields
            for node in nodes:
                if not all(key in node for key in ['id', 'action', 'parameters', 'description']):
                    return False

            # Check that all edges have required fields
            for edge in edges:
                if not all(key in edge for key in ['from', 'to', 'relationship']):
                    return False

            # Check that edge references exist in nodes
            node_ids = {node['id'] for node in nodes}
            for edge in edges:
                if edge['from'] not in node_ids or edge['to'] not in node_ids:
                    return False

            # Check for cycles using NetworkX
            G = nx.DiGraph()
            for node in nodes:
                G.add_node(node['id'])
            for edge in edges:
                G.add_edge(edge['from'], edge['to'])

            # Check if the graph has cycles
            try:
                cycle = nx.find_cycle(G, orientation='original')
                if cycle:
                    rospy.logwarn(f'Detected cycle in action graph: {cycle}')
                    return False
            except nx.NetworkXNoCycle:
                pass  # No cycle found, which is good

            return True

        except Exception as e:
            rospy.logerr(f'Error validating graph structure: {e}')
            return False

    def execute_action_graph(self, graph_data):
        """
        Execute an action dependency graph

        Args:
            graph_data: Dictionary containing nodes and edges

        Returns:
            Boolean indicating success or failure
        """
        rospy.loginfo('Executing action graph')

        # Create NetworkX graph
        G = nx.DiGraph()

        # Add nodes
        for node in graph_data['nodes']:
            G.add_node(node['id'], **node)

        # Add edges
        for edge in graph_data['edges']:
            G.add_edge(edge['from'], edge['to'])

        # Check if the graph is valid (acyclic)
        if not nx.is_directed_acyclic_graph(G):
            rospy.logerr('Action graph contains cycles - cannot execute')
            return False

        # Get execution order using topological sort
        try:
            execution_order = list(nx.topological_sort(G))
            rospy.loginfo(f'Execution order: {execution_order}')
        except nx.NetworkXUnfeasible:
            rospy.logerr('Cannot determine execution order - graph has cycles')
            return False

        # Execute actions in order
        completed = set()
        for node_id in execution_order:
            node = G.nodes[node_id]

            # Check if all dependencies are completed
            predecessors = list(G.predecessors(node_id))
            if not all(pred in completed for pred in predecessors):
                rospy.logwarn(f'Not all dependencies completed for {node_id}')
                continue

            # Execute the action
            success = self.execute_action_node(node)
            if success:
                completed.add(node_id)
                rospy.loginfo(f'Completed action: {node_id}')
            else:
                rospy.logerr(f'Failed to execute action: {node_id}')
                return False

        rospy.loginfo('Action graph executed successfully')
        self.publish_status('Action graph completed successfully')
        return True

    def execute_action_node(self, node):
        """
        Execute a single action node

        Args:
            node: Node dictionary with action details

        Returns:
            Boolean indicating success or failure
        """
        action_id = node.get('id', 'unknown')
        action_type = node.get('action', 'unknown')
        parameters = node.get('parameters', {})
        description = node.get('description', '')

        rospy.loginfo(f'Executing action node {action_id}: {action_type} - {description}')

        if action_type == 'navigate_to':
            return self.execute_navigation_action(parameters)
        elif action_type == 'scan':
            return self.execute_scan_action(parameters)
        elif action_type == 'clean':
            return self.execute_cleaning_action(parameters)
        elif action_type == 'unknown':
            rospy.logwarn(f'Unknown action in node {action_id}')
            return False
        else:
            rospy.logwarn(f'Unsupported action type: {action_type}')
            return False

    def get_environment_context(self):
        """
        Get current environment context

        Returns:
            String describing the environment
        """
        return "indoor environment with furniture, obstacles, and navigation markers"

    def get_robot_capabilities(self):
        """
        Get robot capabilities context

        Returns:
            String describing robot capabilities
        """
        return "navigation, manipulation, perception, voice interaction, basic cleaning tools"

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
            'room_center': (0.0, 0.0, 0.0),
            'home_base': (0.0, 0.0, 0.0)
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
    Main function to demonstrate action graph generation
    """
    generator = ActionGraphGenerator()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down action graph generator")


if __name__ == '__main__':
    main()