# Mobile Robots – Laboratory 2: Odometry

## Overview

This project evaluates robot localization using odometry based on wheel encoder data. The code estimates a robot's position and orientation (`x`, `y`, `θ`) and compares it to ground truth data from ROS2 bag files.

---

## Tasks

### Task 1: Odometry Calculation

- Implements two odometry methods:
  - Using encoder ticks (`calculate_odometry_position`)
  - Using velocities (`calculate_odometry_velocity`)
- Supports input from:
  - Live ROS2 `/joint_states` topic
  - Pre-recorded `.db3` ROS2 bag files

### Task 2: Verification and Comparison

- Compares calculated odometry to `/odom` topic (assumed ground truth)
- Evaluates final position error and visualizes:
  - Computed vs recorded path
  - Errors in position (x, y)
  - Error in orientation (θ)

---

## How to Run

Run the script using your `.db3` ROS2 bag file: follow the following example

```bash
python3 Odometry.py odom_square_right_0.db3

