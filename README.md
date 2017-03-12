# SensorFusionSafetySystem

## Running Dynamixels  
roslaunch dynamixel_driver controller_manager.launch  
roslaunch dynamixel_driver dxl_controller.launch   

Now you can do: rostopic pub -1 /tilt_controller/command std_msgs/Float64 '{data: -0.5}'  
Change tilt_controller to tilt/pan/spindar _controller  
The data is the angle in rads  

The node I will try to make is used by rosrun dynamixel_driver dxl_joints.py