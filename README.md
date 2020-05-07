
**Install following packages**
```
sudo apt-get install ros-melodic-laser-assembler
sudo apt-get install ros-melodic-ros-numy

pip install open3d
pip install sklearn

```

**Move Gazebo Files**

Save MMP30_Simulation directory into the catkin_workspace. 


Next the Appropriate Cotton Plant Model and Ground Model should be moved into the .gazebo model folder

Move contents of directory below into ~/.gazebo directory
MMP30_Simulation/mmp30_gazebo/gazebo_models/



**First Try starting only Rover Simulation in an Empty world**
```
roslaunch mmp30_gazebo mmp30_empty_world.launch

```
Install any necessary packages for which errors might arise at this point


**Start Rover Simulation in Cotton Field**

```
roslaunch mmp30_gazebo mmp30_cotton_field.launch

```

**Start Visualizer**

```
roslaunch mmp30_viz view_robot_sim.launch 
```


**Start Autonavigation Controller**
```
rosrun pure_pursuit_controller pure_pursuit_controller
```

**Start Laser Assembler ROS Package**
```
roslaunch mmp30_gazebo laser_assembler.launch
```

**Start Actuation of the Hokuyo LiDAR**
```
rosrun mmp30_gazebo tilter.py
```

**Takes liDAR snapshots at periods of time**
```
rosrun mmp30_gazebo periodic_snapshotter
```


**Master Navigation control module that shifts between lidar based nav and pure pursuits nav**
```
rosrun mmp30_control NavMaster.py

```


