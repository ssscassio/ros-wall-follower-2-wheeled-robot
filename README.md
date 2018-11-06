# Reactive wall following robot with laser scanner sensor

This project implements a wall-following algorithm in python for an autonomous mobile 2 wheeled robot with a laser scanner sensor using the [Robot Operating System (ROS)](http://www.ros.org/) libraries and [Gazebo](http://gazebosim.org/) as simulator. The proposed wall-following algorithm makes a robot wander at random until a wall is found, then follows the wall - through an implemented proportional control to keep a constant distance from it - in the outside and inside of a “V" and "W" shaped wall, respectively.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Test Environment
- ROS : vx.xx.xx (Melodic)
- Ubuntu: 18.04LTS
- Python: 

### Project structure

#### Catkin Workspace Folder
    .
    ├── catkin_ws               # Catkin Workspace
    │   ├── src                 # Source files 
    │   │   ├── two-wheeled-robot-motion-planning
    │   │   │   ├── scripts     # Folder that contains the follow wall scripts
    │   │   │   └── ...
    │   │   └── ...
    │   ├── devel               # Folder created after compilation
    │   │   ├── setup.bash      # Shell Script to add environment variables to your path
    │   │   └── ...             # etc.
    │   └── build               # Compiled files
    └── simulation_ws

#### Simulation Workspace Folder
    .
    └── simulation_ws           # Simulation Workspace
        ├── src                 # Source files 
        │   ├── two-wheeled-robot-simulation
        │   │   ├── m2wr_description    # Folder that contains the robot definition
        │   │   ├── my_worlds           # Folder that contains world's definitions
        │   │   └── ...
        │   └── ...
        ├── devel               # Folder created after compilation
        │   ├── setup.bash      # Shell Script to add environment variables to your path
        │   └── ...             # etc.
        └── build               # Compiled files
    
### Setup

After clone the project navigate to the project folder
```
cd ros-wall-follower-2-wheeled-robot
```
Navigate to Catkin Workspace and run `catkin_make`
```
cd catkin_ws
catkin_make
```
Return to the project folder 
```
cd ..
```
Navigate to Simulation Workspace and run `catkin_make`
```
cd simulation_ws
catkin_make
```
## Running the tests

To run the tests it will be necessary to use 3 different terminals. Follow the steps starting at project root folder.

### First Terminal
*Do not close this terminal*
1) Run the command to setup the environments variables on this terminal:
```
source simulation_ws/devel/setup.bash
```

2) Choose and launch the World that you want to simulate:

```
roslaunch my_worlds V_world.launch
```
or 
```
roslaunch my_worlds W_world.launch
```

### Second Terminal

1) Run the command to setup the environments variables on this terminal:
```
source simulation_ws/devel/setup.bash
```

2) Spawn the Mobile 2 Wheeled Robot on the World:
```
roslaunch m2wr_description spawn.launch y:=1
```

### Third Terminal
*Do not close this terminal*

1) Run the command to setup the environments variables on this terminal:
```
source catkin_ws/devel/setup.bash
```
2) Run the wall-following script:
```
rosrun two-wheeled-robot-motion-planning follow_wall.py
```
## Built With

* [ROS](http://www.ros.org/) - Set of software libraries and tools used to build the robot
* [Gazebo](http://gazebosim.org/) - Tool used to simulation


## Authors

* [Ana Rafael](https://github.com/SofiaRafael)
* [Cássio Santos](https://github.com/ssscassio)
