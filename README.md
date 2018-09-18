# aeplanner

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

## Setup dependencies

If you do not already have a workspace, set up a new one.

```
mkdir -p catkin_ws/src
cd catkin_ws/src
cd ..
catkin init
catkin build
```

Install dependencies

Octomap
```
sudo apt-get install "ros-kinetic-octomap-*
```

Catkin simple
```
cd catkin_ws/src
git clone git@github.com:catkin/catkin_simple.git
```

## Setup aeplanner

Clone this package (make sure that you are in catkin_ws/src)
```
git clone git@github.com:mseln/aeplanner.git
cd ..
catkin build
```

## Simulation environment

To run the example directly without having to integrate the interface, the kth uav simulation package can be installed, see:

```
https://github.com/danielduberg/kth_uav
```

