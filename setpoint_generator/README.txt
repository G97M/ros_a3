Package: setpoint_generator
-----------------------------------------------
Description:
  This package implements a ROS 2 node that continuously publishes wheel velocity
  commands in a fixed sequence. It is designed for a differential-drive robot RELbot or
  RELbot Simulator that listens to  /input/left_motor/setpoint_vel and
  /input/right_motor/setpoint_vel topics  

  The node cycles through:
    1. An initial delay (no motion for 5 seconds),
    2. Forward motion for 5 seconds,
    3. Stopping for 3 seconds,
    4. Backward motion for 5 seconds,
    5. Another stop for 3 seconds,
    and then repeats from forward motion.

Inputs:
  (None)
    This node does not subscribe to any external topics. It simply publishes commands.

Outputs:
  /input/left_motor/setpoint_vel
    Type: example_interfaces/msg/Float64
    Description: The setpoint velocity (in rad/s) for the left motor.

  /input/right_motor/setpoint_vel
    Type: example_interfaces/msg/Float64
    Description: The setpoint velocity (in rad/s) for the right motor.

Parameters:
  (None)
    This node does not currently expose any parameters. All timing and velocity
    values are hard-coded in the source.

Run:
  1. Make sure your workspace is built and sourced:
       source install/setup.bash

  2. Run the node:
       - ros2 launch setpoint_generator relbot_test.launch.py

Core Components:
  - timer_callback():
      A function triggered at 50 Hz (every 20 ms). It checks the current state
      (INITIAL_DELAY, FORWARD, STOP1, BACKWARD, STOP2) and how long the node has
      been in that state. Depending on the state:
        * It may publish wheel velocities for forward or backward motion,
        * It may publish zero velocities for stopping,
        * It transitions to the next state after the allotted time has passed.
      Once STOP2 is complete, the cycle restarts at FORWARD.

Notes:
  - The node logs messages indicating each phase transition (e.g., "Starting forward motion").
  - You can verify the published commands with:
       ros2 topic echo /input/left_motor/setpoint_vel
       ros2 topic echo /input/right_motor/setpoint_vel
  - If used with the RELbot Simulator or a compatible robot, you will see it move forward,
    stop, backward, stop, then repeat.
  - To adjust durations or velocities, edit the source code where the state transitions
    and velocity assignments occur (e.g., 5-second delays, ±1.0 rad/s wheel speeds).
