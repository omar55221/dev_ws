# Navigation Command Documentation

This document provides a list of commands to run the navigation stack. Each command should be executed in a separate terminal after sourcing the `source install/setup.bash` file in the `/dev_ws` directory.

## Terminal 1

**Command:**
```bash
ros2 launch void_bot launch_sim.launch.py world:=src/void_bot/worlds/obstacles.world
```

**Description:**
This command launches the simulation environment using the `void_bot` package. It specifies the world file `obstacles.world` located in the `src/void_bot/worlds` directory.

## Terminal 2

**Command:**
```bash
rviz2 -d src/void_bot/config/main.rviz
```

**Description:**
This command opens RViz2 with a pre-configured display settings file `main.rviz` located in the `src/void_bot/config` directory. RViz2 is used for visualizing the robot's state and environment.

## Terminal 3

**Command:**
```bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/void_bot/config/mapper_params_online_async.yaml use_sim_true:=true
```

**Description:**
This command launches the SLAM (Simultaneous Localization and Mapping) toolbox in online asynchronous mode. It uses the parameters specified in `mapper_params_online_async.yaml` and enables simulation time.

## Terminal 4

**Command:**
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

**Description:**
This command starts the navigation stack using the `nav2_bringup` package. It enables the use of simulation time, which is necessary when running in a simulated environment.

terminal 5

python3 send_waypoints.py
