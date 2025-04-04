Package: brightness_detector
-----------------------------------------------
Description:
  This package implements a brightness detection node using OpenCV and CV Bridge.
  The node subscribes to an image stream, converts the image to grayscale, computes the average brightness, 
  and compares it to a dynamically adjustable threshold. Based on this comparison, it publishes a status 
  message indicating whether the light is ON or OFF.

Inputs:
  /image
    Type: sensor_msgs/msg/Image
    Description: The input image stream from a camera or video source.

Outputs:
  /light_status
    Type: std_msgs/msg/String
    Description: A status message indicating whether the brightness exceeds the threshold ("Light is ON") 
                 or not ("Light is OFF").

Parameters:
  double threshold
    Description: Brightness threshold value (0-255) used to decide if the scene is bright enough.
    Default: 100.0
    Range: 0.0 to 255.0
    Note: The parameter is dynamically configurable at runtime with a parameter change callback.

Run:
  In a terminal, you can run the node using one of the following commands:
    ros2 run brightness_detector brightness_detector

Core Components:
  image_callback():
    - Converts the incoming BGR image to grayscale.
    - Computes the average brightness of the grayscale image.
    - Compares the computed brightness with the current threshold.
    - Publishes a std_msgs/msg/String message ("Light is ON" or "Light is OFF") based on the comparison.
  
  parameter_callback():
    - Handles dynamic updates to the 'threshold' parameter.
    - Validates that the new value is within the acceptable range (0-255).
    - Updates the internal threshold variable safely using a mutex to ensure thread safety.
  
  Use of OpenCV and CV Bridge:
    - OpenCV is used for image processing tasks (color conversion and brightness computation).
    - CV Bridge converts ROS image messages to OpenCV format, enabling seamless image processing.

Note:
  - Ensure that the ROS 2 environment is properly sourced.
  - The node uses mutexes to protect shared variables during parameter updates and image processing.
  - Adjust the threshold parameter at runtime as needed to accommodate different lighting conditions.
