# Troubleshooting Guide

This guide provides solutions to common issues encountered when working with ROS 2.

## Common Issues

### "command not found" for ROS 2 commands
**Solution**: Ensure you've sourced the ROS 2 environment: `source /opt/ros/humble/setup.bash`

### Python packages not found
**Solution**: Make sure you've installed the Python development tools and sourced your workspace

### Permission errors during installation
**Solution**: Use `sudo` for system package installations, but not for workspace operations

### Nodes can't communicate between terminals
**Solution**: Ensure ROS 2 environment is sourced in each terminal

### Workspace build fails
**Solution**:
1. Make sure you're in the workspace root directory
2. Run `colcon build` with the correct options
3. Check for any compilation errors in the output

## Getting Help

- Check the official ROS 2 documentation: https://docs.ros.org/en/humble/
- Visit the ROS answers forum: https://answers.ros.org/
- Join the ROS discourse: https://discourse.ros.org/