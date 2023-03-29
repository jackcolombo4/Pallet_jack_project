# ros-gps-plugin
A Gazebo Plugin that is able to publish the measurement of  a gazebo sensor on a ROS topic.

## Parameters
Include in the plugin tag the folllowing parameters:
- multiPathProbability: probability of getting a translated measurement from the gps sensor.
- lossProbability: probability of getting loss of signal from sensor.
- maxLongTranslation: max amount of translation in the longitude. The translation is between -maxLongTranslation and +maxLongTranslation.
- maxLatTranslation: the same as before for the latitude. 

## Compile and run
- Create the build folder

```
cd build
cmake ../
make
```
- Link the plugin

```
export GAZEBO_PLUGIN_PATH=$HOME/ros-gps-plugin-path/faultygps/build:$GAZEBO_PLUGIN_PATH
```

- Launch gazebo inside ros

```
roslaunch gazebo_ros empty_world.launch
```

- Add the gps to the world and check if the topic /fix has been advertised


