Package: object_detector
-----------------------------------------------
Description:
  This package implements an object detection node using OpenCV and CV Bridge.
  The node processes incoming images by converting them to grayscale and applying a binary threshold.
  It then computes image moments to determine the center of the detected object and its area.
  If the detected region's area is above a specified minimum, the object's center coordinates and area are published.
  
Inputs:
  /image
    Type: sensor_msgs/msg/Image
    Description: The input image stream from a camera or video source.

Outputs:
  /object_position
    Type: geometry_msgs/msg/PointStamped
    Description: A message containing:
      - point.x: X-coordinate of the object's center
      - point.y: Y-coordinate of the object's center
      - point.z: The area of the detected object (indicating the object's size)

Parameters:
  threshold
    Type: integer
    Default: 200
    Range: 0-255
    Description: The brightness threshold used for converting the grayscale image to a binary image.
  
  min_area
    Type: integer
    Default: 100
    Description: The minimum area (in pixels) required for a detected region to be considered a valid object.

Run:
  In a terminal, run the following commands:
    ros2 run object_detector object_detector
    

Core Components:
  configure_parameters():
    - Declares the 'threshold' and 'min_area' parameters with descriptive constraints.
    - Retrieves initial parameter values for use in image processing.

  parameter_callback():
    - Handles dynamic parameter updates at runtime.
    - Validates new parameter values (ensuring the threshold is between 0 and 255, and min_area is positive).
    - Updates the internal parameters safely using mutex locks.

  image_callback():
    - Converts the incoming image from BGR to grayscale.
    - Applies a binary threshold based on the current threshold parameter.
    - Computes image moments to calculate the object's center (x, y) and area.
    - Publishes a geometry_msgs/msg/PointStamped message on the /object_position topic.
    - If the detected area is below min_area, the message indicates an invalid detection.

Note:
  - Ensure that your ROS 2 environment is properly sourced.
  - The node uses mutexes to protect shared resources during parameter updates and image processing.
  - Adjust the parameters at runtime as needed to accommodate varying lighting conditions or object sizes.
