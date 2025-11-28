# IMU Dead-Reckoning with Xsens MTi-630  
Reconstructing Robot Motion Using Madgwick Filter + ZUPT + Drift Correction

---

**Author:** Priyam Gupta            
**Neptun Code:** KXHGEA

---

## Overview
This project reconstructs a robot's trajectory using only IMU data recorded from an **Xsens MTi-630** sensor.  
Dead-reckoning is performed using orientation estimation (Madgwick filter), frame transformations, gravity compensation, Zero-Velocity Updates (ZUPT), and drift-corrected velocity/position integration.

Two datasets are processed, corresponding to two robot motion trials inside a building:
- [Dataset 1 (small motion)](imu_data/xsens_mti_630_imu_first.txt)
- [Dataset 2 (longer motion)](imu_data/xsens_mti_630_imu_second.txt)

---

## Features
- Load & preprocess Xsens IMU TXT logs  
- Convert **SampleTimeFine** timestamps into seconds  
- Estimate accelerometer, gyroscope, magnetometer biases  
- Convert IMU measurements **from ENU -> NED**  
- Apply **Madgwick 9-DoF orientation filter**  
- Convert acceleration from **body frame -> world frame**  
- Subtract gravity to compute **linear acceleration**  
- Detect stationary phases using **ZUPT**  
- Integrate acceleration -> velocity (with resets)  
- Correct long-term drift using interpolation  
- Integrate velocity -> position  
- Plot final **2D trajectories** for Dataset 1 & 2  

---

## Core Pipeline

### 1. Load & Preprocess Data
- Remove comment lines  
- Parse IMU channels  
- Convert timestamps  
- Convert gyro from °/s to rad/s  

### 2. Bias Estimation
Using first 2 seconds (robot stationary):
- Accelerometer bias  
- Gyroscope bias  
- Magnetometer bias  

### 3. Coordinate Transform
Convert Xsens output from **ENU -> NED** so it matches the navigation frame used by Madgwick.

### 4. Orientation Estimation
Madgwick **updateMARG()** is applied to obtain quaternion attitude in NED.

### 5. Body -> World Acceleration
Apply rotation matrix:
```
a_world = R_b2w * a_body
```

### 6. Gravity Removal
Because XSens already removes gravity, `linacc = acc_world_raw` is used directly.

### 7. ZUPT - Zero Velocity Update
Stationary frames are detected using:
- |acc| < threshold  
- |gyro| < threshold  

Velocity is reset at those indices.

### 8. Drift Correction
Linear interpolation between first & last stationary segments.

### 9. Position Integration
Velocity integrated again to obtain displacement.

---

## Trajectory Results

### Dataset 1 Behavior
- Robot moves forward  
- Makes a gradual curved path  
- Returns near the origin  
- Small drift remains as expected for IMU-only motion  

### Dataset 2 Behavior
- Longer run with more turning  
- Robot descends a hallway  
- Turns sharply  
- Moves back and forth  
- Ends far from starting point due to long-distance drift  

(Plots included in notebook.)

---

## Data
Place the Xsens TXT data files in:
```
data/
```

---

## Acknowledgements
This project uses:
- **AHRS library** for Madgwick filtering  
- **SciPy** for quaternion and rotation operations  
- Original Xsens MTi-630 IMU datasets  

---
