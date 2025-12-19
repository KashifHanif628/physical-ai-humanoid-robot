---
title: Chapter 3 Exercises - Humanoid URDF Fundamentals
sidebar_label: Chapter 3 Exercises
description: Exercises to practice working with URDF models
keywords: ros2, urdf, exercises, robot, model, humanoid
---

# Chapter 3 Exercises - Humanoid URDF Fundamentals

## Exercise 1: Understanding URDF Structure

**Difficulty**: Intermediate
**Estimated Time**: 30 minutes
**Prerequisites**: Understanding of XML and basic URDF concepts

### Objective
Analyze the provided URDF model and identify its components.

### Steps
1. Examine the simplified NAO-like URDF model from Chapter 3
2. Identify all the links in the model
3. Identify all the joints in the model
4. For each joint, determine its type and range of motion
5. Identify which joints are for arms, legs, and head
6. Explain how the links are connected in a tree structure

### Expected Outcome
- Complete list of all links and their properties
- Complete list of all joints with their types and limits
- Understanding of the robot's kinematic structure
- Ability to trace the link-joint relationships

### Hints
- Start from the base_link and trace connections to all other links
- Look for parent-child relationships in joint definitions
- Pay attention to the joint types and their limitations

---

## Exercise 2: Modifying URDF Properties

**Difficulty**: Intermediate
**Estimated Time**: 40 minutes
**Prerequisites**: Understanding of URDF structure

### Objective
Modify the URDF model to change the robot's appearance.

### Steps
1. Create a copy of the simplified NAO URDF file
2. Increase the size of the head by changing the sphere radius from 0.05 to 0.07
3. Make the arms longer by increasing the cylinder length from 0.1 to 0.15
4. Change the color of the torso from white to blue
5. Adjust the joint limits to allow for a wider range of motion
6. Validate the modified URDF file for proper syntax

### Expected Outcome
- Modified URDF file with changed dimensions and colors
- Valid XML structure without errors
- Larger head and longer arms
- Changed color properties

### Hints
- Be careful with units (meters in URDF)
- Remember to update both visual and collision geometries
- Joint limits should be physically realistic

---

## Exercise 3: Adding a New Link

**Difficulty**: Advanced
**Estimated Time**: 60 minutes
**Prerequisites**: Understanding of URDF structure and joint relationships

### Objective
Add a new link (e.g., a camera) to the robot model.

### Steps
1. Choose a location to add a new sensor link (e.g., on the head)
2. Create a new link element with appropriate geometry (e.g., small box for camera)
3. Create a joint that connects the new link to an existing link (e.g., head)
4. Position the new link appropriately using the joint's origin
5. Add appropriate inertial properties for the new link
6. Test that the new link is properly connected and positioned

### Expected Outcome
- A new sensor link added to the URDF model
- Proper connection to the existing robot structure
- Correct positioning and orientation
- Valid URDF syntax

### Hints
- Use fixed joints if the sensor doesn't move relative to its parent
- Consider the physical size of the sensor when choosing geometry
- Remember to update the robot's total mass with the new link's mass