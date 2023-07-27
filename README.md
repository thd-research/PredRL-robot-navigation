# PredRL-robot-navigation

Predictive reinforcement learning: mapless navigation method for mobile robot.
This repository contains sources for results reproducing of paper submitted to Journal of Intelligent Manufacturing.

## Setting up

### Environment

Follow steps directories

```
mkdir misis
cd misis
git clone git@github.com:thd-research/PredRL-robot-navigation.git
```

### Install Docker

You need to go into the downloaded folder and go into the docker folder.



``` cd ros-2023-thd/docker```
    

If you don't have an Nvidia graphics card, run the command:

    
```./install_docker.bash```
    
    
If you have an Nvidia graphics card, run the command:

   
   ```./install_docker.bash -n```
   
    
### Build Docker
We already have a ready-made environment and you just need to build it:

```sudo ./build_docker.sh```
    
Or if you have an Nvidida video card:

 ```sudo ./build_docker.sh -n```

### 4. Run Docker

To execute the docker container use command:

   
   ```sudo ./run_docker.sh```

Or if you have an Nvidida video card:

  
  ```sudo ./run_docker.sh -n```

    
If you need additional terminal inside of the Docker open new window in the terminal (Ctrl+Shift+T) and use command

    
    sudo ./into_docker.sh
    

### Setting up the environment

Open docker-container

```
cd docker
bash run_docker.sh 
```

Inside of the docker navigate to the workspace and build the workspace

```
cd misis/PredRL-robot-navigation/turtlebot_ws
catkin_build
source devel/setup.bash
```

## Rcognita

This repository is a snapshot of [Rcognita](https://github.com/AIDynamicAction/rcognita.git).
```rcognita``` is a flexibly configurable framework for agent-enviroment simulation with a menu of predictive and safe reinforcement learning controllers. A detailed documentation is available under the link above.

To execute benchamarking, run the gazebo simulation world (here, and further all commands must be executed in Docker!)

```
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```

Now, you can launch the ```rcognita```. Make sure that your path is ```/misis/PredRL-robot-navigation```. 
Next,

```cd rcognita/presets
python3 main_3wrobot_ros_obst.py --Nactor 6 --pred_step_size_multiplier 8 --dt 0.1 --ctrl_mode MPC
```

### Settings

Some key settings are described below (full description is available via
``-h`` option).


| Parameter                     | Type    | Description                                            |
| ------------------------------|:-------:| :-----------------------------------------------------:|
| ``ctrl_mode``                 | string  | Controller mode (MPC, SQL, RQL)                        |
| ``dt``                        | number  | Controller sampling time                               |
| ``Nactor``                    | integer | Horizon length (in steps) for predictive controllers   |
| ``pred_step_size_multiplier`` | integer | Prediction step size multiplier                        |













Further details regarding ```Rcognita``` you can find [here](https://github.com/thd-research/PredRL-robot-navigation/tree/main/rcognita).
