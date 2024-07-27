# The Perrobot simulation - Python code 
This repository contains all the files and code needed to simulate the quadruped robot Perrobot based on the Quadruped Robots from [Open Dynamic Robot Initiative](https://github.com/open-dynamic-robot-initiative/). This siulation was built in the context of an internship at ... using [Gazebo](https://gazebosim.org/home) and [ROS](https://www.ros.org/). The software runs on [ROS noetic](http://wiki.ros.org/noetic) and [Ubuntu 20.04](http://www.releases.ubuntu.com/20.04/).

This README provides step-by-step instructions to set up and run the simulation for both the Perrobot 8dof and 12dof.


## Prerequisites

Before you begin, ensure you have the following installed:

- Ubuntu (20.04 recommended)
- ROS Noetic

## Setup

Follow these steps to setup :

1. **Create a works space and clon the repository in it**

```sh
cd ~
mkdir perrobot_test_ws && cd perrobot_test_ws
git clone https://github.com/Andy-Mod/perrobot.git

```

2. **Initializing the workspace **

```sh

cd src && catkin_init_workspace
cd .. && catkin_make
source ~/perrobot_test_ws/devel/setup.bash >> ~/.bashrc
source ~/.bashrc

```

3. **Intalling python dependences **

```sh
cd ~/perrobot_test_ws/
pip install -r requirements.txt

```
## Run

Two robot are simulated here : 

### The Perrobot 8dof 

```sh
roslaunch perrobot run_gazebo.launch

```

### The Perrobot 12dof 
```sh

roslaunch perrobot run_gazebo_12dof.launch 

```

## Interface 

Upon running the simulation in both cases, here is the interface : 



