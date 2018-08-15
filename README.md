# SensorFusionSafetySystem

## Running Dynamixels  
roslaunch dynamixel_driver controller_manager.launch  
roslaunch dynamixel_driver dxl_controller.launch   

Now you can do: rostopic pub -1 /tilt_controller/command std_msgs/Float64 '{data: -0.5}'  
Change tilt_controller to tilt/pan/spindar _controller  
The data is the angle in rads  

The node I will try to make is used by rosrun dynamixel_driver dxl_joints.py


## Running all in Simulation:
roslaunch post_gazebo post_gazebo.launch 

roslaunch post_control post_control.launch 

roslaunch post_description post_rviz.launch 

rosrun observer observer.py 

rosrun detector detector.py

rosrun lidar_monitor lidar_monitor.py 

rosrun xtion_monitor xtion_monitor.py

rosrun fake_proximity fake_proximity.py
