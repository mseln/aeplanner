# Autonomous Exploration Planner - aeplanner

aeplanner is an exploration planning package for 3d environments. It subscribes to an [OctoMap](https://octomap.github.io/) and will propose waypoints that maximizes the information gain while minimizing the traversed distance. For a more detailed description see our paper.

<div align="center">
  <a href="https://youtu.be/Mg93ojV5rC8">
    <img
     src="https://img.youtube.com/vi/Mg93ojV5rC8/hqdefault.jpg" 
     alt="Efficient Autonomous Exploration Planning of Large Scale 3D-Environments" 
     style="width:100%;">
  </a>
</div>
    
Use aeplanner?
If you are using aeplanner, please cite our paper **Efficient Autonomous Exploration Planning of Large-Scale 3-D Environments** in IEEE Robotics and Automation Letters, vol. 4, no. 2, pp. 1699-1706, April 2019.

BibTeX:
```
@ARTICLE{8633925,
    author={M. {Selin} and M. {Tiger} and D. {Duberg} and F. {Heintz} and P. {Jensfelt}},
    journal={IEEE Robotics and Automation Letters},
    title={Efficient Autonomous Exploration Planning of Large-Scale 3-D Environments},
    year={2019},
    volume={4},
    number={2},
    pages={1699-1706},
    keywords={Planning;Sensors;Uncertainty;Drones;Mobile robots;Path planning;Search and rescue robots;motion and path planning;mapping},
    doi={10.1109/LRA.2019.2897343},
    ISSN={2377-3766},
    month={April},}
```


## System Requirements
This repository has been tested with: 
```
Ubuntu 16.04
ROS Kinetic (Desktop full installation)
catkin tools
catkin simple
OctoMap
```
it might work with other setups, but no guarantees are given.

## Assumptions
1. You are running `Ubuntu 16.04`
2. You are using bash, zsh is also possible with some [modifications](https://github.com/mseln/aeplanner/wiki/Use-zsh-instead-of-bash).
3. You have `ros-kinetic-desktop-installed`, otherwise follow installation instructions [here](http://wiki.ros.org/kinetic/Installation/Ubuntu)
4. You have created a workspace in `~/catkin_ws`, without running `catkin_make`. If in another location [these](https://github.com/mseln/aeplanner/wiki/Other-location-than-~-catkin_ws) things need to be modified.
5. You have installed [`catkin tools`](https://catkin-tools.readthedocs.io/en/latest/installing.html)


## Installation of dependencies

1. You have installed [`octomap`](http://wiki.ros.org/octomap) if not install it by running `sudo apt-get install "ros-kinetic-octomap-*"` or install from source.

2. Catkin simple is needed to build the rtree package
```
cd ~/catkin_ws/src
git clone https://github.com/catkin/catkin_simple.git
```

3. PIGain depends on rtree which in turn needs libspatialindex (note this rtree is different from the one above).
```
pip install rtree
sudo apt-get install libspatialindex-dev
```

## Setup workspace
If you do not already have a workspace, set up a new one.
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin build
```

## Setup aeplanner

Clone this package (make sure that you are in catkin_ws/src)
```
cd ~/catkin_ws/src/
git clone https://github.com/mseln/aeplanner.git
cd ..
catkin build
```

## Simulation environment

To run the example directly without having to integrate the interface, the kth uav simulation package can be installed, see:

```
https://github.com/danielduberg/kth_uav
```

Read the Assumptions and follow the install instructions on that page.

#### Troubleshooting installation of kth_uav

If you get `couldn't find python module jinja2:` install jinja2 by running:
```
sudo apt-get install python-jinja2
```

If you get `No rule to make target '/home/rpl/catkin_ws/src/kth_uav/Firmware/Tools/sitl_gazebo/PROTOBUF_PROTOC_EXECUTABLE-NOTFOUND', needed by 'Groundtruth.pb.cc'.  Stop.`, install libprotobuf by running:

```
sudo apt-get install protobuf-compiler
```

If you get `ImportError: No module named future` install it by running:
```
pip install future
```

If you get `Could not find a package configuration file provided by "geographic_msgs" with any of the following names:` install it by running:
```
sudo apt-get install ros-kinetic-geographic-msgs
```

If you get `Could NOT find GeographicLib (missing: GeographicLib_LIBRARIES GeographicLib_INCLUDE_DIRS)` fix it by installing:
```
sudo apt-get install libgeographic-dev
```

If gazebo is complaining about `GeographicLib exception: File not readable /usr/share/GeographicLib/geoids/egm96-5.pgm`, install  `geographiclib-tools` and run the install script again.
```
sudo apt-get install geographiclib-tools
sudo ./kth_uav/mavros/mavros/scripts/install_geographiclib_datasets.sh
```


When running `roslaunch simulation simulation.launch` if you get `Arg xml is <arg default="$(find mavlink_sitl_gazebo)/worlds/empty.world" name="world"/>` fix it by changing line 14 and 15 in `kth_uav/Firmware/launch/mavros_posix_sitl.launch` to
```
    <arg name="world"/>  
    <arg name="sdf"/>
```
do the same with line in file 12 and 13 `kth_uav/Firmware/launch/posix_sitl.launch`.

