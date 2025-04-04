1 Assignment set 1

Requirements : 
Cam2image package - relbot_simulator

Assignment 1.1 
-----------------------------------------------
1.1.1 
1. open a cmd and run videoserver.py script (WSL)
2.open 3 terminals
3.terminal 1 --> ros2 run cam2image_vm2ros cam2image --ros-args -p depth:=1 -p history:=keep_all --params-file src/cam2image_vm2ros/config/cam2image.yaml
4.terminal 2 --> ros2 run image_tools showimage
5.terminal 3 --> ros2 topic hz /image 
---------------------------------------------------------
1.1.2
1. open a cmd and run videoserver.py script (WSL)
2.open 3 terminals
3.terminal 1 --> ros2 run cam2image_vm2ros cam2image --ros-args -p depth:=1 -p history:=keep_all --params-file src/cam2image_vm2ros/config/cam2image.yaml
4.terminal 2 --> ros2 run ros2 run light_status_detector light_status_node
5-terminal 3 --> ros2 topic echo /light_status
---------------------------------------------------------
1.1.3
1. open a cmd and run videoserver.py script (WSL)
2.open 4 terminals
3.terminal 1 --> ros2 run cam2image_vm2ros cam2image --ros-args -p depth:=1 -p history:=keep_all --params-file src/cam2image_vm2ros/config/cam2image.yaml
4.terminal 2 --> ros2 run brightness_detector brightness_detector_node --ros-args -p threshold:=150.0
5.terminal 3 --> ros2 topic echo /light_status
6.terminal 4 --> in the first run 
		 ros2 param set /brightness_detector threshold 80.0
		 in the second run
		 ros2 param set /brightness_detector threshold 300.0
---------------------------------------------------------
1.1.4

bright object detection

1. open a cmd and run videoserver.py script (WSL)
2.open 4 terminals
3.terminal 1 --> ros2 run cam2image_vm2ros cam2image --ros-args -p depth:=1 -p history:=keep_all --params-file src/cam2image_vm2ros/config/cam2image.yaml
4.terminal 2 --> ros2 run object_detector object_detector_node --ros-args \
  -p threshold:=200 \
  -p min_area:=50
5.terminal 3 --> ros2 topic echo /object_position
6.terminal 4 --> in the first run 
		 ros2 param set /object_detector threshold 180
		 in the second run
		 ros2 param set /object_detector min_area 30

colored object detection

1.open a cmd and run videoserver.py script (WSL)
2.open 4 terminals
3.terminal 1 --> ros2 run cam2image_vm2ros cam2image --ros-args -p depth:=1 -p history:=keep_all --params-file src/cam2image_vm2ros/config/cam2image.yaml
4.terminal 2 --> ros2 run color_object_detector color_object_detector_node
5.terminal 3 --> ros2 topic echo /object_position
6.terminal 4 --> in the first run 
		 ros2 param set /color_object_detector lower_hsv "[160, 100, 100]"
		 in the second run
		 ros2 param set /color_object_detector upper_hsv "[180, 255, 255]"

---------------------------------------------------------------------------------------

Assignment 1.2 
-----------------------------------------------
1.2.1

setpoint_generator

1.open a cmd and run videoserver.py script (WSL)
2.open 5 terminals
3.terminal 1 --> ros2 run cam2image_vm2ros cam2image --ros-args -p depth:=1 -p history:=keep_all --params-file src/cam2image_vm2ros/config/cam2image.yaml
4.terminal 2 --> ros2 launch setpoint_generator relbot_test.launch.py
5.terminal 3 --> ros2 topic echo /output/robot_pose
6.terminal 4 --> ros2 topic echo /input/left_motor/setpoint_vel  
7.terminal 5 --> ros2 run rqt_plot rqt_plot /output/robot_pose/pose/position/x /input/left_motor/setpoint_vel/data		 
		 
-----------------------------------------------------------------------------------------
1.2.2

sequence_controller_extended

1.open a cmd and run videoserver.py script (WSL)
2.open 4 terminals
3.terminal 1 --> ros2 run cam2image_vm2ros cam2image --ros-args -p depth:=1 -p history:=keep_all --params-file src/cam2image_vm2ros/config/cam2image.yaml
4.terminal 2 --> ros2 launch sequence_controller_extended sequence_controller_extended.launch.py
5.terminal 3 --> ros2 topic echo /output/robot_pose
6.terminal 4 --> ros2 topic echo /object_position 

--------------------------------------------------------------------------------------------

