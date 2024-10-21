#!/bin/bash

# Source the ROS 2 workspace environment
source /dev_ws/install/setup.bash

# Trap to handle cleanup on Ctrl+C
stop_simulation() {
    echo "Stopping all processes..."
    kill $SIM_PID $RVIZ_PID $SLAM_PID $NAV_PID
    exit 0
}

# Trap SIGINT (Ctrl+C) and call the stop_simulation function
trap stop_simulation SIGINT

# Launch the simulation environment in the background
echo "Launching simulation environment..."
ros2 launch void_bot launch_sim.launch.py world:=src/void_bot/worlds/obstacles.world &
SIM_PID=$!
sleep 50  # Give the simulation time to start

# Launch RViz in the background
echo "Launching RViz2..."
rviz2 -d src/void_bot/config/main.rviz &
RVIZ_PID=$!
sleep 20  # Allow RViz to fully start

# Start the SLAM toolbox in the background
echo "Starting SLAM toolbox..."
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/void_bot/config/mapper_params_online_async.yaml use_sim_time:=true &
SLAM_PID=$!
sleep 20  # Allow SLAM to initialize

# Launch the navigation stack in the background
echo "Starting navigation stack..."
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true &
NAV_PID=$!
sleep 20  # Allow navigation stack to initialize

# Run the waypoint sending Python script and wait for it to finish
echo "Sending waypoints..."
python3 send_waypoints.py &
WAYPOINT_PID=$!
wait $WAYPOINT_PID  # Wait for waypoints to be sent

# After the waypoints are sent, monitor the log for completion message
echo "Monitoring for waypoints completion..."
ros2 topic echo /waypoint_follower/log | grep -m 1 "Completed all 4 waypoints requested"

# Once the log message is received, stop all processes
echo "Waypoints completed, stopping simulation..."
stop_simulation
