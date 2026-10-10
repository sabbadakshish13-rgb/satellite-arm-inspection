# Satellite Inspection Robotic Arm Simulation

## 1. Project Objective
Develop a robotic arm simulation capable of detecting
and tracking a satellite inspection target using
camera-based feedback.

## 2. Software Environment
- Ubuntu 24.04 (WSL2)
- ROS 2 Jazzy
- Gazebo Harmonic
- Python 3
- OpenCV

## 3. Implemented Features
- Robotic arm simulation in Gazebo
- Zero-gravity simulation environment
- Three-joint robotic arm model
- Camera-based red target detection
- Automatic target tracking
- Joint position control
- Moving inspection target simulation
- Tracking error measurement and visualization

## 4. Experimental Results

### Stationary Target
- Measurements: 23
- Mean overall tracking error: 8.82 pixels

### Moving Target
- Target displacement: 50 mm
- Measurements: 66
- Mean overall tracking error: 8.73 pixels
- Maximum overall error: 13.34 pixels
- Samples within alignment tolerance: 65/66

Note: Moving-target statistics include stationary periods.

## 5. Important Project Files
- src/satellite_arm_description/urdf/satellite_arm.urdf
- src/satellite_arm_description/worlds/inspection_zero_gravity.sdf
- experiment_results/
- ~/auto_marker_tracking.py
- ~/moving_inspection_target.py

## 6. Current Limitations
- Tracking is based on image coordinates.
- Joint angles are commanded values.
- Physical end-effector accuracy has not been validated.
- Experiments were conducted in simulation only.

## 7. Future Improvements
- Add inverse kinematics
- Implement end-effector positioning
- Add collision detection
- Develop autonomous inspection trajectories
- Evaluate tracking under different target speeds
