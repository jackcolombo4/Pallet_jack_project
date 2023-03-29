# gazebo_pallet_jack
Design of an autonomous pallet jack using gazebo


## Prerequisites: 
ROS kinetic, Gazebo and turtlebot3_teleop installed.

## Files: 
	- Mass-CenterOfGravity.xlsx: used for the computation of the center of gravity and the mass and transformation into the gazebo coordinate frame.
	- Project1.world: sdf file containing the ground plane, sun, robot model, configuration of the differential drive plugin, reference to the compiled library of the encoder plugin and physics parameters. 
	- encoderPlugin folder: c++ plugin source code, CMakeLists.txt and build folder that, after compiling, will contain all the files created and the resulting library libencoder.so.


## Instructions
Install the ros differential drive plugin:
```
sudo apt-get install ros-kinetic-diff-drive-controller 
```

To control the robot from the keyboard there's the need to install teleop-twist:
```
sudo apt-get install ros-kinetic-teleop-twist-keyboard
```

If apt does not find the package:
```
apt-cache search ros-kinetic-teleop
```

and find another package with a similar name.

### Compiling the plugin
Move or link the gazebo_pallet_jack directoy into your catkin src directory
move:
```
mv /path_to_gazebo_pallet_jack /path_to_catkinws/src
```
or link:
```
ln -s  /path_to_gazebo_pallet_jack /path_to_catkinws/src
```

And compile:
```
cd path_to_catkinws
catkin_make

```

### Add the model to the Gazebo folder
Add the directory to the model folder of gazebo, in this way gazebo can load the pallet_jack:

```
cd ~/.gazebo/models
ln -s ~/path_to_model_folder
```

Add the directory of the box to the model folder of gazebo, in this way gazebo can load the box:

```
cd ~/.gazebo/models
ln -s ~/path_to_folder/200k_box
```

### Run

#### Standard way

To run:
```
roslaunch gazebo_pallet_jack pallet.launch
```

If you want to see a little animation:
```
roslaunch gazebo_pallet_jack pallet_animation.launch
```

if the commands don't work try:
```
source /path_to_catkin_workspace/deved/setup.bash
```

and execute the launch command again.

#### Control the robot

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=pallet_jack/cmd_vel
```

Where robot_0 must be substituted with the name of the robot.
In this way teleop publishes on the right topic.

### Laser Scan 
Laser Scan publish messages of type sensor_msgs/LaserScan.msg
The sensor class is GpuRaySensor.
Reference: 
- http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/LaserScan.html
- http://answers.ros.org/question/198843/need-explanation-on-sensor_msgslaserscanmsg/
- http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/group__gazebo__sensors.html

## Reference: 
- http://wiki.ros.org/stdr_simulator/Tutorials/Teleop%20with%20teleop_twist_keyboard

- http://www.theconstructsim.com/how-to-build-a-differential-drive-simulation/

- http://answers.gazebosim.org/question/12366/clarification-on-moving-joint-with-model-plugin/

## Author
- Giacomo Colombo