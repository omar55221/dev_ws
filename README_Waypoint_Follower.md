# Using Nav2 Waypoint Follower with Your Simulation

This guide provides detailed instructions on how to use the Nav2 Waypoint Follower with your simulation setup.

## Prerequisites

- Ensure you have Docker and Visual Studio Code installed with the Remote - Containers extension.
- Make sure your development container is set up and running.

## Launching the Simulation

1. **Source the Setup File**:
   Open a terminal and source the setup file to configure your environment:
   ```bash
   source /dev_ws/install/setup.bash
   ```

2. **Launch the Simulation Environment**:
   Open a new terminal and run the following command to start the simulation in Gazebo:
   ```bash
   ros2 launch void_bot launch_sim.launch.py world:=src/void_bot/worlds/obstacles.world
   ```

3. **Visualize with RViz2**:
   In another terminal, execute the following command to start RViz2 with the appropriate configuration:
   ```bash
   rviz2 -d src/void_bot/config/main.rviz
   ```

## Using the Waypoint Follower

1. **Launch the Navigation Stack**:
   Open a new terminal and run the following command to launch the navigation stack, including the waypoint follower:
   ```bash
   ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
   ```

2. **Send Waypoints**:
   You can send waypoints to the waypoint follower node. This can be done through a script or a node that publishes to the `/waypoints` topic. Ensure your waypoints are defined in a YAML file or programmatically.

## Configuration Details

- **Parameters File**: The navigation stack uses parameters from `nav2_params.yaml`. Ensure this file is correctly configured for your simulation environment.
- **Behavior Tree**: The default behavior tree XML file is `navigate_w_replanning_and_recovery.xml`, located in the `nav2_bt_navigator` package.

## Conclusion

By following these steps, you can effectively use the Nav2 Waypoint Follower with your simulation setup. Ensure all configurations are tailored to your specific environment for optimal performance.
