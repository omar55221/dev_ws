#!/bin/bash

# Source the ROS 2 workspace environment
source /dev_ws/install/setup.bash

# Launch the simulation environment in the background (as if in a separate terminal)
echo "Launching simulation environment..."
ros2 launch void_bot launch_sim.launch.py world:=src/void_bot/worlds/obstacles.world &
SIM_PID=$!
sleep 50  # Give the simulation time to start

# Launch RViz in the background (another terminal equivalent)
echo "Launching RViz2..."
rviz2 -d src/void_bot/config/main.rviz &
RVIZ_PID=$!
sleep 20  # Allow RViz to fully start

# Start the SLAM toolbox in the background (another terminal equivalent)
echo "Starting SLAM toolbox..."
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/void_bot/config/mapper_params_online_async.yaml use_sim_time:=true &
SLAM_PID=$!
sleep 20  # Allow SLAM to initialize

# Launch the navigation stack in the background (another terminal equivalent)
echo "Starting navigation stack..."
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true &
NAV_PID=$!
sleep 20  # Allow navigation stack to initialize

# Run the waypoint sending Python script (final step)
echo "Sending waypoints..."
python3 send_waypoints.py &
WAYPOINT_PID=$!

# Wait for all background processes to complete
wait $SIM_PID $RVIZ_PID $SLAM_PID $NAV_PID $WAYPOINT_PID
