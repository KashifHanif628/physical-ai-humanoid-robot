---
title: "LLM Planning Exercises"
sidebar_label: "Chapter 2 Exercises: LLM Planning"
description: "Exercises for LLM-driven cognitive planning"
keywords: ["exercises", "llm", "planning", "cognitive", "practice"]
---

# LLM Planning Exercises

## Exercise 1: LLM Setup and Basic Planning (Beginner)
- **Difficulty**: Beginner
- **Estimated Time**: 45 minutes
- **Prerequisites**: Basic Python knowledge, understanding of LLM concepts

### Objective
Set up LLM integration and implement basic task decomposition functionality.

### Steps
1. Install LLM libraries (LangChain, OpenAI)
2. Configure API keys and connections
3. Create a simple prompt template for task decomposition
4. Test with basic commands like "Move forward"
5. Verify the LLM generates appropriate action sequences

### Expected Outcome
A working LLM integration that can decompose simple commands into action sequences.

### Solution
Use LangChain to create a chain that takes natural language and outputs structured action plans.

### Hints
- Start with simple, well-defined commands
- Use structured output formats for predictable results
- Test with various command phrasings

### Validation Criteria
- LLM responds to commands consistently
- Output follows expected structure
- Basic commands are properly decomposed

---

## Exercise 2: Action Graph Generation (Intermediate)
- **Difficulty**: Intermediate
- **Estimated Time**: 60 minutes
- **Prerequisites**: Exercise 1 completed, understanding of dependency graphs

### Objective
Implement action graph generation with dependency management for complex tasks.

### Steps
1. Create a system to represent action dependencies
2. Implement graph construction from LLM output
3. Add support for parallel action execution
4. Test with multi-step commands like "Go to kitchen and bring water"
5. Verify dependencies are properly managed

### Expected Outcome
An action graph system that can handle complex tasks with dependencies and parallel execution.

### Solution
Use graph data structures to represent action relationships and execute in dependency order.

### Hints
- Consider using networkx or similar library for graph operations
- Implement validation to ensure no circular dependencies
- Plan for failure scenarios in the graph execution

### Validation Criteria
- Action graphs are properly constructed
- Dependencies are respected during execution
- Parallel actions execute correctly when possible

---

## Exercise 3: Complete Planning Integration (Advanced)
- **Difficulty**: Advanced
- **Estimated Time**: 120 minutes
- **Prerequisites**: Exercises 1 and 2 completed, ROS 2 knowledge

### Objective
Integrate LLM planning with ROS 2 action execution for a complete cognitive planning system.

### Steps
1. Connect LLM planner to ROS 2 action clients
2. Implement plan validation before execution
3. Add safety checks and constraint validation
4. Integrate with voice processing system from Chapter 1
5. Test complete end-to-end pipeline with complex commands

### Expected Outcome
A complete system that takes natural language, generates plans, and executes them safely.

### Solution
Combine LLM planning with ROS 2 action orchestration and safety validation.

### Hints
- Implement comprehensive error handling and recovery
- Add timeouts and safety limits to all actions
- Consider real-time performance requirements

### Validation Criteria
- Natural language commands are properly planned
- Action sequences execute safely and correctly
- System handles errors and edge cases appropriately