# Data Model: Digital Twin Module (Gazebo & Unity)

**Feature**: 2-digital-twin-gazebo-unity
**Model Version**: 1.0
**Created**: 2025-12-18

## Overview

This documentation module focuses on content organization rather than traditional data storage. The "data model" describes the structure of the educational content and associated simulation resources.

## Content Entities

### Chapter
A major section of the module containing related concepts and examples.

**Attributes**:
- id: String (unique identifier, e.g., "chapter-1-physics-simulation")
- title: String (display title for the chapter)
- description: String (brief overview of chapter content)
- objectives: Array of String (learning objectives)
- prerequisites: Array of String (required knowledge/skills)
- duration: Number (estimated completion time in minutes)
- content: String (main content in Markdown format)
- exercises: Array of Exercise (hands-on activities)
- simulationExamples: Array of SimulationExample (executable simulation examples)

**Relationships**:
- Contains many SimulationExample
- Contains many Exercise
- Preceded by previous Chapter (for sequential learning)

### SimulationExample
A runnable simulation example demonstrating digital twin concepts.

**Attributes**:
- id: String (unique identifier)
- title: String (descriptive title)
- description: String (what the example demonstrates)
- type: String (simulation type: "gazebo", "unity", "integration")
- files: Array of String (list of simulation files)
- dependencies: Array of String (required packages/libraries)
- expectedBehavior: String (description of expected simulation behavior)
- fileLocation: String (path where example files are stored)
- executionInstructions: String (how to run the simulation)

**Relationships**:
- Belongs to one Chapter
- May reference multiple DigitalTwinConcept

### Exercise
A hands-on learning activity for the learner to complete.

**Attributes**:
- id: String (unique identifier)
- title: String (descriptive title)
- description: String (detailed instructions)
- difficulty: String (e.g., "beginner", "intermediate", "advanced")
- estimatedTime: Number (time needed in minutes)
- prerequisites: Array of String (what is needed to complete the exercise)
- steps: Array of String (sequential instructions)
- expectedOutcome: String (what the learner should achieve)
- hints: Array of String (optional guidance)
- solution: String (reference solution)

**Relationships**:
- Belongs to one Chapter
- May reference multiple SimulationExample

### DigitalTwinConcept
A fundamental digital twin concept covered in the module.

**Attributes**:
- id: String (unique identifier, e.g., "digital-twin", "physics-simulation", "sensor-simulation")
- name: String (concept name)
- definition: String (clear definition)
- purpose: String (why this concept exists)
- usage: String (how it's typically used)
- examples: Array of SimulationExample (demonstrating the concept)

**Relationships**:
- Referenced by multiple SimulationExample
- Covered in one or more Chapter

### SimulationEnvironment
A virtual environment for robot simulation.

**Attributes**:
- id: String (unique identifier)
- name: String (environment name)
- description: String (what the environment represents)
- physicsEngine: String (physics engine used: "ode", "bullet", "dart")
- objects: Number (count of static objects)
- fileLocation: String (path to world file)
- configuration: String (physics and environment parameters)

**Relationships**:
- Belongs to Chapter 1
- Referenced by multiple SimulationExample (for simulation scenarios)

## Content Validation Rules

### Chapter Validation
- Title must be 5-100 characters
- Objectives must be specific and measurable
- Content must be in valid Markdown format
- Must contain at least one SimulationExample
- Duration estimate must be reasonable (5-120 minutes)

### SimulationExample Validation
- Files must exist and be accessible
- Dependencies must be clearly specified
- Execution instructions must be complete
- Expected behavior must be described
- Simulation must run without errors

### Exercise Validation
- Difficulty level must be specified
- Prerequisites must be clearly defined
- Steps must be sequential and clear
- Expected outcome must be achievable
- Estimated time must be realistic

### DigitalTwinConcept Validation
- Definition must be clear and concise
- Purpose must be explained
- Must have at least one simulation example
- Name must be standard terminology

## State Transitions (for content development)

### Chapter States
1. **Draft**: Initial content creation
2. **Review**: Under review by technical experts
3. **Revised**: Feedback incorporated
4. **Complete**: Ready for publication
5. **Published**: Available to learners

### SimulationExample States
1. **Concept**: Idea for example defined
2. **Implemented**: Simulation created and tested
3. **Validated**: Confirmed working in target environment
4. **Integrated**: Included in chapter content
5. **Published**: Available to learners

## Relationships Summary

- Chapter (1) contains many SimulationExample (M)
- Chapter (1) contains many Exercise (M)
- SimulationExample (M) references many DigitalTwinConcept (M) (many-to-many via documentation)
- Exercise (M) may reference many SimulationExample (M)
- SimulationEnvironment (M) belongs to Chapter (1) - specifically Chapter 1

## File Structure Mapping

The data model maps to the following file structure:

```
docs/
├── digital-twin/
│   ├── chapter-1-physics-simulation.md
│   ├── chapter-2-sensor-simulation.md
│   ├── chapter-3-unity-integration.md
│   ├── exercises/
│   │   ├── chapter-1-exercises.md
│   │   ├── chapter-2-exercises.md
│   │   └── chapter-3-exercises.md
│   └── examples/
│       ├── gazebo-worlds/
│       │   ├── basic-world.world
│       │   └── humanoid-simulation.world
│       ├── sensor-configs/
│       │   ├── lidar-sensor.xacro
│       │   ├── depth-camera.xacro
│       │   └── imu-sensor.xacro
│       ├── simulation-examples/
│       │   ├── humanoid-gazebo-example.py
│       │   └── sensor-data-pipeline.py
│       └── unity-scenes/
│           ├── robot-visualization.unity
│           └── unity-scripts/
│               └── ros-bridge.cs
```