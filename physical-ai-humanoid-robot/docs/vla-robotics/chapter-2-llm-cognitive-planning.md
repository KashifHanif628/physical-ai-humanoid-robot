---
title: "LLM-Driven Cognitive Planning"
sidebar_label: "Chapter 2: LLM-Driven Cognitive Planning"
description: "Learn about LLM-driven cognitive planning and task decomposition"
keywords: ["llm", "planning", "cognitive", "task decomposition", "language models"]
---

# LLM-Driven Cognitive Planning

## Introduction

Large Language Models (LLMs) excel at understanding natural language commands and can be leveraged for cognitive planning in robotics. This chapter explores how LLMs can decompose complex tasks into executable action sequences for humanoid robots.

## Translating Language into Plans

LLMs can understand high-level natural language commands and break them down into structured action sequences. This cognitive planning capability is essential for autonomous robotic systems.

### Chain-of-Thought Reasoning

LLMs can use chain-of-thought reasoning to plan complex tasks step by step:

1. **Understand the goal**: Parse the natural language command
2. **Identify subtasks**: Break down the main task into smaller components
3. **Sequence actions**: Order the subtasks logically
4. **Consider constraints**: Account for safety and environmental factors
5. **Generate execution plan**: Create a sequence of executable actions

### Example: "Clean the Room" Decomposition

A natural language command like "Clean the room" can be decomposed into:

```
1. Navigate to the room
2. Scan the room for objects
3. Identify objects that need cleaning (dust, trash, etc.)
4. Plan cleaning path efficiently
5. Execute cleaning actions (sweep, vacuum, organize)
6. Return to home position
```

## Task Decomposition and Sequencing

Effective task decomposition requires understanding both the linguistic structure and the physical capabilities of the robot.

### Planning Algorithms

Several approaches can be used for LLM-based planning:

1. **Hierarchical Task Networks (HTN)**: Decompose tasks into subtasks recursively
2. **Behavior Trees**: Represent tasks as tree structures with conditions
3. **Finite State Machines**: Define states and transitions for task execution
4. **Graph-based Planning**: Represent tasks as nodes in a graph with dependencies

### Example: LLM Planning Node

```python
from langchain_openai import ChatOpenAI
from langchain.prompts import PromptTemplate
import json

class LLMPlanner:
    def __init__(self):
        self.llm = ChatOpenAI(model="gpt-3.5-turbo")

        self.planning_prompt = PromptTemplate.from_template(
            """Convert this natural language command into a structured action plan.
            Command: {command}
            Environment: {environment}
            Robot capabilities: {capabilities}

            Return a JSON list of actions with the following structure:
            [
                {{
                    "action": "action_name",
                    "parameters": {{"param1": "value1", "param2": "value2"}},
                    "description": "Brief description"
                }}
            ]

            Actions should be executable by a ROS 2 robot."""
        )

    def generate_plan(self, command, environment=None, capabilities=None):
        """
        Generate a structured action plan from natural language command
        """
        if environment is None:
            environment = "standard room environment"
        if capabilities is None:
            capabilities = "navigation, manipulation, perception"

        chain = self.planning_prompt | self.llm
        response = chain.invoke({
            "command": command,
            "environment": environment,
            "capabilities": capabilities
        })

        try:
            # Extract JSON from response
            json_start = response.content.find('[')
            json_end = response.content.rfind(']') + 1

            if json_start != -1 and json_end != 0:
                plan_json = response.content[json_start:json_end]
                plan = json.loads(plan_json)
                return plan
            else:
                # If no JSON found, return a simple plan
                return self.fallback_plan(command)
        except json.JSONDecodeError:
            return self.fallback_plan(command)

    def fallback_plan(self, command):
        """
        Fallback plan generation if LLM fails
        """
        # Simple rule-based fallback
        if "clean" in command.lower():
            return [
                {"action": "navigate_to_room", "parameters": {}, "description": "Go to the room to clean"},
                {"action": "scan_environment", "parameters": {}, "description": "Scan for cleaning targets"},
                {"action": "execute_cleaning", "parameters": {}, "description": "Clean the area"},
                {"action": "return_home", "parameters": {}, "description": "Return to charging station"}
            ]
        return [{"action": "unknown_command", "parameters": {"command": command}, "description": "Unable to parse command"}]
```

## ROS 2 Action Orchestration

LLMs can generate action sequences that are orchestrated using ROS 2 actions for reliable execution.

### Action Graph Generation

LLMs can generate dependency graphs that represent the relationships between different actions:

```python
class ActionGraphExecutor:
    def __init__(self):
        self.action_clients = {}
        self.dependencies = {}

    def execute_plan(self, action_plan):
        """
        Execute an action plan with dependency management
        """
        # Build dependency graph
        self.build_dependencies(action_plan)

        # Execute actions respecting dependencies
        completed = set()

        while len(completed) < len(action_plan):
            # Find actions whose dependencies are satisfied
            ready_actions = []
            for i, action in enumerate(action_plan):
                if i in completed:
                    continue

                deps_satisfied = True
                for dep_idx in self.dependencies.get(i, []):
                    if dep_idx not in completed:
                        deps_satisfied = False
                        break

                if deps_satisfied:
                    ready_actions.append((i, action))

            # Execute ready actions in parallel
            for idx, action in ready_actions:
                success = self.execute_single_action(action)
                if success:
                    completed.add(idx)
                else:
                    # Handle failure - could retry, skip, or abort
                    return False

        return True

    def execute_single_action(self, action):
        """
        Execute a single action using ROS 2
        """
        action_name = action["action"]
        parameters = action["parameters"]

        # Map action name to ROS 2 action client
        if action_name in self.action_clients:
            client = self.action_clients[action_name]
            # Execute the action with parameters
            return self.send_action_goal(client, parameters)
        else:
            rospy.logerr(f"Unknown action: {action_name}")
            return False
```

## Integration with Voice Processing

LLM planning can be integrated with the voice processing system from Chapter 1 to create a complete pipeline:

```
Voice Command → Whisper Transcription → Intent Classification → LLM Planning → Action Execution
```

### Example Integration

```python
class IntegratedVLAPlanner:
    def __init__(self):
        self.whisper_processor = WhisperVoiceProcessor()
        self.llm_planner = LLMPlanner()
        self.action_executor = ActionGraphExecutor()

    def process_voice_command(self, voice_command):
        """
        Process a voice command through the complete VLA pipeline
        """
        # Step 1: Parse intent from voice command
        intent = self.parse_intent(voice_command)

        # Step 2: Generate plan using LLM
        plan = self.llm_planner.generate_plan(
            command=voice_command,
            environment=self.get_environment_state(),
            capabilities=self.get_robot_capabilities()
        )

        # Step 3: Execute the plan
        success = self.action_executor.execute_plan(plan)

        return success
```

## Best Practices

### Structured Outputs
- Use consistent JSON formats for LLM outputs
- Implement validation for generated action sequences
- Include error handling in generated plans

### Safety Considerations
- Embed safety constraints in planning prompts
- Validate action sequences before execution
- Implement timeout and emergency stop mechanisms

### Performance Optimization
- Cache common command-to-plan mappings
- Use appropriate LLM models for real-time requirements
- Implement plan validation and optimization

## Troubleshooting

For common issues, refer to the [troubleshooting guide](./common/troubleshooting-guide.md).

## Summary

LLM-driven cognitive planning enables robots to understand complex natural language commands and decompose them into executable action sequences. By combining LLMs with ROS 2 action orchestration, we can create intelligent systems that can plan and execute complex tasks. The next chapter will explore how to integrate these components into a complete autonomous humanoid system.