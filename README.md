# JetsonCar-Simulation
Simulation environment for the Jetson Car project using Gazebo and ROS


# Cloning
To set up the simulation environment you need to clone the necessary repositories into an existing or new catkin workspace.
Follow the steps below to set up a new catkin workspace and clone:
```bash
mkdir -p ~/jetsoncar_simulation_ws/src
cd ~/jetsoncar_simulation_ws/src
catkin_init_workspace
git clone https://github.com/mindThomas/JetsonCar-Gazebo
git clone https://github.com/mindThomas/JetsonCar-ROS
git clone https://github.com/mindThomas/realsense_gazebo_plugin
cd ..
rosdep install --from-paths src --ignore-src -r -y
```

# Building
Build the project with catkin build
```bash
cd ~/jetsoncar_simulation_ws
catkin build
source devel/setup.bash
```

# Launch simulation
The Gazebo simulation can be launched with
```bash
roslaunch jetsoncar_gazebo gazebo.launch
```

# Reset simulation
The Gazebo simulation/world can be reset without having to restart Gazebo by calling:
```bash
rosservice call /gazebo/reset_world
```

# Debugging
## View TF frames
Published frames (TF's) can be viewed with (generates a PDF)
```bash
rosrun tf view_frames
```

## Convert Xacro to URDF
Another way would be to convert the xacro file to urdf and then `urdf_to_graphiz`
```bash
xacro JetsonCar-Simulation/jetsoncar_description/urdf/jetsoncar.xacro >> jetsoncar.urdf
urdf_to_graphiz jetsoncar.urdf
evince jetsoncar.pdf
```



# Notes
`joint_state_publisher` is only for testing purposes. A node in a real system or Gazebo for simulation should provide the current actual joint angles by updating all tf's.

Both joint transmission (actuators) and joint controllers will have to be initialized: http://wiki.ros.org/urdf/Tutorials/Using%20a%20URDF%20in%20Gazebo
