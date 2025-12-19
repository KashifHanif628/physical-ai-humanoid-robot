---
title: "Isaac AI Brain Troubleshooting Guide"
sidebar_label: "Troubleshooting Guide"
description: "Common issues and solutions for Isaac AI Brain module"
keywords: ["troubleshooting", "debugging", "issues", "solutions", "isaac"]
---

# Isaac AI Brain Troubleshooting Guide

This guide provides solutions to common issues encountered when working with Isaac tools.

## Isaac Sim Issues

### Performance Problems
- **Issue**: Low frame rates in simulation
- **Solution**: Reduce scene complexity or upgrade GPU
- **Alternative**: Use Isaac Sim in headless mode for data generation

### Installation Issues
- **Issue**: Missing dependencies
- **Solution**: Ensure NVIDIA repositories are properly added
- **Command**: `sudo apt update && sudo apt install nvidia-isaac-common-repos`

## Isaac ROS Issues

### Installation Problems
- **Issue**: Missing Isaac ROS packages
- **Solution**: Verify repository setup and package installation
- **Command**: `ros2 pkg list | grep isaac`

### VSLAM Performance
- **Issue**: VSLAM not running in real-time
- **Solution**: Check hardware acceleration settings and GPU resources

## Nav2 Navigation Issues

### Navigation Safety
- **Issue**: Robot not navigating safely
- **Solution**: Check humanoid-specific constraints in configuration
- **Check**: Verify local and global planner parameters

### Path Planning
- **Issue**: Suboptimal path planning for bipedal robots
- **Solution**: Adjust planner parameters for humanoid-specific requirements