---
title: "Chapter 2 Exercises - Sensors in Simulation"
sidebar_label: "Chapter 2 Exercises"
description: "Exercises to practice sensor simulation in Gazebo"
keywords: [sensors, simulation, gazebo, lidar, camera, imu, exercises, robotics]
---

# Chapter 2 Exercises - Sensors in Simulation

## Exercise 1: LiDAR Sensor Configuration

**Difficulty**: Intermediate
**Estimated Time**: 40 minutes
**Prerequisites**: Understanding of SDF format and sensor configuration

### Objective
Configure a LiDAR sensor with realistic parameters and analyze the data.

### Steps
1. Add a LiDAR sensor to a robot model (or create a simple robot with a LiDAR)
2. Configure the sensor with realistic parameters:
   - 360-degree horizontal field of view
   - 720 samples (0.5 degree resolution)
   - Range: 0.1m to 20m
   - Update rate: 10Hz
3. Add Gaussian noise with 0.01m standard deviation
4. Create a world with obstacles (boxes, cylinders) for the LiDAR to detect
5. Launch the simulation and subscribe to the LiDAR topic to analyze the data
6. Write a simple node to process the LiDAR data and detect obstacles

### Expected Outcome
- Successfully configured LiDAR sensor publishing data
- LiDAR data shows obstacles in the environment
- Node processes and analyzes the LiDAR data correctly
- Understanding of how noise affects sensor readings

### Hints
- Use `ros2 topic echo` to inspect the LiDAR data
- Check the `sensor_msgs/LaserScan` message format
- Consider how different parameters affect performance and accuracy

---

## Exercise 2: Camera and IMU Integration

**Difficulty**: Intermediate
**Estimated Time**: 50 minutes
**Prerequisites**: Understanding of depth cameras and IMU sensors

### Objective
Configure both a depth camera and IMU sensor on the same robot and integrate their data.

### Steps
1. Add a depth camera to your robot with the following parameters:
   - Resolution: 640x480 pixels
   - Field of view: 60 degrees
   - Update rate: 30Hz
   - Add realistic noise model
2. Add an IMU sensor to the same robot with:
   - Accelerometer and gyroscope noise models
   - Update rate: 100Hz
3. Create a ROS 2 node that subscribes to both sensor streams
4. Synchronize the sensor data based on timestamps
5. Visualize the data (show camera image and IMU orientation simultaneously)

### Expected Outcome
- Both sensors publishing data correctly
- Synchronized sensor data processing
- Understanding of multi-sensor data integration
- Visualization of combined sensor information

### Hints
- Use `message_filters` for timestamp synchronization
- Check ROS 2 documentation for sensor message types
- Consider time delays between sensor measurements

---

## Exercise 3: Sensor Fusion and Noise Analysis

**Difficulty**: Advanced
**Estimated Time**: 75 minutes
**Prerequisites**: Understanding of sensor data processing and filtering

### Objective
Implement a basic sensor fusion algorithm that combines data from multiple sensors and analyze noise characteristics.

### Steps
1. Create a robot with LiDAR, camera, and IMU sensors
2. Design a simple sensor fusion node that combines sensor data
3. Implement a basic Kalman filter or complementary filter to fuse IMU and camera data
4. Add artificial delays to simulate network latency
5. Analyze the noise characteristics of each sensor type
6. Compare fused data accuracy vs individual sensor accuracy
7. Document the effectiveness of sensor fusion in reducing noise

### Expected Outcome
- Working sensor fusion algorithm
- Quantified noise reduction through fusion
- Understanding of how different sensors complement each other
- Analysis of fusion algorithm performance

### Hints
- Start with simple fusion (e.g., averaging overlapping measurements)
- Use ROS 2's `tf2` library for coordinate transformations
- Consider the frequency differences between sensor types
- Evaluate fusion performance under different conditions