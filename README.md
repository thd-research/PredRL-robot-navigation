# PredRL-robot-navigation

Predictive reinforcement learning: mapless navigation method for mobile robot.
This repository contains sources for results reproducing of paper submitted to Journal of Intelligent Manufacturing.

## Setting up

### Environment

Follow steps directories

```
mkdir turtlebot_ws
cd turtlebot_ws
git clone https://github.com/thd-research/PredRL-robot-navigation.git
```

Then, cd into the repo and download git submodules

```
cd PredRL-robot-navigation
git submodule init
git submodule update
```

### Install Docker

You need to go into the docker folder.



```cd docker```
    

If you don't have an Nvidia graphics card, run the command:

    
```./install_docker.bash```
    
    
If you have an Nvidia graphics card, run the command:

   
   ```./install_docker.bash -n```
   
    
### Build Docker
We already have a ready-made environment and you just need to build it:

```sudo ./build_docker_cuda.sh```
    
Or if you have an Nvidida video card:

```sudo ./build_docker_cuda.sh -n```

### 4. Run Docker

To execute the docker container use command:

   
   ```sudo ./run_docker.sh```

Or if you have an Nvidia video card:

  
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
cd turtlebot_ws/PredRL-robot-navigation/turtlebot_ws
catkin_make
source devel/setup.bash
```

## Rcognita

This repository is a snapshot of [Rcognita](https://github.com/AIDynamicAction/rcognita.git).
```rcognita``` is a flexibly configurable framework for agent-enviroment simulation with a menu of predictive and safe reinforcement learning controllers. A detailed documentation is available under the link above.

To execute benchmarking, run the gazebo simulation world (here, and further all commands must be executed in Docker!)

```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```

Now, you can launch `rcognita`

```
cd /turtlebot_ws/PredRL-robot-navigation/rcognita/presets
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

## Sources

1. This repository reflects the results of the work presented in the paper [Predictive reinforcement learning: map-less navigation method for mobile robot](https://link.springer.com/epdf/10.1007/s10845-023-02197-y?sharing_token=9J6qFaLJK8zlSeK8qLZNEfe4RwlQNchNByi7wbcMAY57w8Mfz1J8LlAq2EfEWCpoY-POnUOX83e-aS6Tl6RrOTGyBfSKSkaln5CkSZ38SxWPuKmr5fV63i9fXyhFPGlJiC9brh5lcPucxTDbQQiii7Dmg08v3kaRZ0H_ptlottk%3D)
2. [Robotis TurtleBot3 Repository](https://github.com/ROBOTIS-GIT/turtlebot3)
3. [Rcognita](https://github.com/AIDynamicAction/rcognita)

## Related literature




Please, give us a credit if you use our sources for your research project :)

```
@article{osinenko2023generalized,
  title={A generalized stacked reinforcement learning method for sampled systems},
  author={Osinenko, Pavel and Dobriborsci, Dmitrii and Yaremenko, Grigory and Malaniya, Georgiy},
  journal={IEEE Transactions on Automatic Control},
  year={2023},
  publisher={IEEE}
}

@article{dobriborsci2022experimental,
  title={An experimental study of two predictive reinforcement learning methods and comparison with model-predictive control},
  author={Dobriborsci, Dmitrii and Osinenko, Pavel and Aumer, Wolfgang},
  journal={IFAC-PapersOnLine},
  volume={55},
  number={10},
  pages={1545--1550},
  year={2022},
  publisher={Elsevier}
}

@article{osinenko2021effects,
  title={Effects of sampling and prediction horizon in reinforcement learning},
  author={Osinenko, Pavel and Dobriborsci, Dmitrii},
  journal={IEEE Access},
  volume={9},
  pages={127611--127618},
  year={2021},
  publisher={IEEE}
}
```



