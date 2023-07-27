# PredRL-robot-navigation

Predictive reinforcement learning: mapless navigation method for mobile robot.
This repository contains sources for results reproducing of paper submitted to Journal of Intelligent Manufacturing.

## Instructions

### Environment

Follow steps directories

```mkdir misis
cd misis
git clone git@github.com:thd-research/PredRL-robot-navigation.git
```

### Install Docker

You need to go into the downloaded folder and go into the docker folder.


    ```cd ros-2023-thd/docker```

If you don't have an Nvidia graphics card, run the command:

    ```./install_docker.bash```
    
If you have an Nvidia graphics card, run the command:

   ``` ./install_docker.bash -n```
    

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

    ```sudo ./into_docker.sh```

