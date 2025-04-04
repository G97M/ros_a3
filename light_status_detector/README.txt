Package: light_status_node
-----------------------------------------------
Description:
  This package implements a node that determines the light status of a scene using image processing. 
  The node subscribes to an image stream, converts the image to grayscale, computes the average brightness, 
  and then compares it to a fixed threshold value. Depending on the result, it publishes a string message 
  indicating whether the light is ON or OFF.

Inputs:
  /image
    Type: sensor_msgs/msg/Image
    Description: The input image stream from a camera or video source.

Outputs:
  /light_status
    Type: std_msgs/msg/String
    Description: A message indicating the light status:
                 - "Light is ON" if the average brightness is above the threshold.
                 - "Light is OFF" if the average brightness is below or equal to the threshold.

Run:
  In a terminal, run the node with:
    ros2 run light_status_node light_status_node

Parameters:
  The node uses a fixed threshold value (hard-coded in the code) of 100.0.
  Note: To modify the threshold, the source code must be changed and the node rebuilt.

Core Components:
  image_callback():
    - Converts the incoming image from BGR to grayscale using OpenCV.
    - Computes the average brightness of the grayscale image.
    - Compares the computed brightness with the fixed threshold.
    - Publishes a std_msgs/msg/String message ("Light is ON" or "Light is OFF") based on the comparison.
  
  Main Function:
    - Initializes the ROS 2 node.
    - Spins the node to continuously process incoming image messages.
    - Shuts down the ROS 2 environment upon exit.

Note:
  - Ensure that the ROS 2 environment is properly sourced before running the node.
  - The node uses cv_bridge to convert ROS image messages to OpenCV format for processing.
  - Debug logging is enabled for detailed brightness and status information.
