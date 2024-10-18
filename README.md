# Workspace Overview and Simulation Deep Dive

This document provides a comprehensive overview of the `dev_ws` workspace, detailing the setup, structure, and functionality of the various components involved, including the development container.

## Development Environment

This project is designed to run within a development container, which provides a consistent and isolated environment for development. The `.devcontainer` setup ensures that all necessary packages and environment configurations are automatically installed and applied.

### Opening the Development Container

1. **Prerequisites**:
   - Ensure you have Docker and Visual Studio Code installed with the Remote - Containers extension.

2. **Open the Development Container**:
   - Open the project directory in Visual Studio Code.
   - Click on the green icon in the bottom-left corner and select "Reopen in Container" to start the development container.

## Workspace Structure

The workspace is organized into several key directories and files:

- **src/void_bot/launch**: Contains launch scripts for setting up the simulation, SLAM, and navigation stacks.
- **src/void_bot/config**: Holds configuration files for various components, such as RViz and SLAM parameters.
- **src/void_bot/worlds**: Includes world files used in the simulation environment.
- **build/**: Contains build artifacts and configuration files.
- **install/**: Includes setup scripts and installed package files.
- **log/**: Stores logs from build and runtime processes.

## Simulation Setup

The simulation is set up using multiple launch files, each responsible for different aspects of the environment and functionality.

### Launch Files

1. **launch_sim.launch.py**:
   - Sets up the simulation environment using Gazebo.
   - Includes the robot state publisher, joystick control, and twist multiplexer.
   - Spawns the robot entity in the Gazebo simulation.

2. **online_async_launch.py**:
   - Launches the SLAM toolbox in asynchronous mode.
   - Uses parameters from `mapper_params_online_async.yaml`.
   - Configures the SLAM node to use simulation time.

3. **navigation_launch.py**:
   - Launches the navigation stack with nodes for controlling, planning, and navigating.
   - Uses parameters from `nav2_params.yaml`.
   - Configures behavior trees for navigation tasks.

## Running the Simulation

To run the simulation, follow these steps:

1. **Source the Setup File**:
   - Open a terminal and source the setup file:
     ```bash
     source /dev_ws/install/setup.bash
     ```

2. **Launch the Simulation Environment**:
   - Open a new terminal and run:
     ```bash
     ros2 launch void_bot launch_sim.launch.py world:=src/void_bot/worlds/obstacles.world
     ```

3. **Visualize with RViz2**:
   - In another terminal, execute:
     ```bash
     rviz2 -d src/void_bot/config/main.rviz
     ```

4. **Start SLAM Toolbox**:
   - Open a new terminal and run:
     ```bash
     ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/void_bot/config/mapper_params_online_async.yaml use_sim_true:=true
     ```

5. **Launch Navigation Stack**:
   - Finally, open another terminal and run:
     ```bash
     ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
     ```

## Conclusion

This workspace provides a robust simulation environment for testing and developing robotic navigation and SLAM capabilities. By following the setup and execution steps, you can explore the full functionality of the `void_bot` package within the `dev_ws` workspace.
