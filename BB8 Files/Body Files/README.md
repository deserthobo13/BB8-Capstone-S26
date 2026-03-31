BB-8 Droid Control System
Overview
This repository contains the core control software for a self-balancing BB-8 astromech droid. The system utilizes a Raspberry Pi, an Adafruit BNO055 IMU, a PCA9685 servo driver, and a PS5 DualSense controller to manage internal pendulum stability, drive locomotion, and head puppeteering.

The codebase has been refactored into a modular, object-oriented architecture to ensure a strict, non-blocking 50Hz control loop necessary for inverted pendulum stability.

Hardware Components
Compute: Raspberry Pi (using pigpio for hardware-accurate PWM)

Sensors: Adafruit BNO055 IMU (I2C)

Actuators: * 3x DC Motors (Main Drive & Steering via Phase/Enable motor drivers)

5x Servos (Internal Swing Pendulum, Head Forward/Backward, Head Side-to-Side, Head Rotation)

Input: PS5 DualSense Controller (Bluetooth via pygame)

Architecture & Module Breakdown
The software is divided into distinct, non-blocking classes to ensure the main controller loop can maintain a steady 50Hz refresh rate.

PS5_Controller.py (Main Orchestrator)
The entry point of the application. It handles the Bluetooth connection to the PS5 controller, manages the system state (e.g., test modes vs. active drive), and runs the primary dynamic timing loop. By tracking elapsed time per cycle and sleeping only for the remainder of the target 0.02s loop time, it ensures the PID and Dead Reckoning math receive accurate delta time (dt) variables.

Movement_Functions.py (Hardware Abstraction Layer)
The BB8Movement class handles all direct hardware interfacing for motors, servos, and power relays.

Balance & Head Leveling: Contains the core update_balance() method, which applies the PID correction to the internal swing servos instantly for stability. It simultaneously applies an Exponential Moving Average (EMA) filter to the head leveling servos for organic, droid-like movement.

Drive & Steering: Clamps motor inputs to a safe 30% maximum speed factor to prevent over-torquing the pendulum.

IMU.py (Sensor Management)
The IMUSensor class abstracts the BNO055 hardware. To minimize latency and prevent I2C clock stretching, it features a single update() method that pulls all Euler angles (Pitch, Roll, Yaw) and linear acceleration in one pass. These values are stored as class attributes for instant retrieval by the control loop.

PID.py (Stability Math)
A lightweight PIDController class. It includes integral anti-windup (clamping the integral term between -10.0 and +10.0) to prevent the droid from over-correcting if physically restrained or stuck. It also includes a reset() method to clear the integral memory before the drive loop is enabled, preventing massive dt spikes.

Dead_Reckoning.py (Telemetry)
The DeadReckoning class integrates Z-axis acceleration to estimate current velocity and distance traveled. It operates as a state machine integrated directly into the main 50Hz loop. It features custom timeout logic to zero out velocity if no acceleration is detected for 0.5 seconds, fixing infinite coasting errors.

Usage
Ensure the PS5 controller is connected via Bluetooth to the Raspberry Pi.

Start the pigpiod daemon.

Run the main controller script:

Bash
python3 PS5_Controller.py
Select 4 from the terminal menu to enter PS5 Control Mode and activate the system relays.

Controls:

L-Stick Y-Axis: Forward / Backward Drive

D-Pad Left / Right: Internal Steering

L1 / R1: Rotate Head Left / Right

X Button: Reset all motors to zero

PS Button: Emergency Stop / Shut down safely
