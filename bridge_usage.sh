#!/bin/bash

echo "ROS 1 Bridge Setup Guide"
echo "======================="
echo ""

echo "This Docker environment provides:"
echo "- ROS 1 Noetic"
echo "- ROS 2 Humble"  
echo "- ROS 1 Bridge"
echo "- Your state estimator packages"
echo ""

echo "Usage Options:"
echo ""

echo "1. Build the image:"
echo "   docker-compose -f docker-compose-bridge.yml build"
echo ""

echo "2. Start interactive container:"
echo "   docker-compose -f docker-compose-bridge.yml run --rm ros-bridge"
echo ""

echo "3. Start bridge only (in background):"
echo "   docker-compose -f docker-compose-bridge.yml up bridge-only"
echo ""

echo "Inside the container:"
echo ""

echo "4. Build your ROS 2 workspace:"
echo "   cd /root/ros2_ws"
echo "   colcon build --packages-select state_estimator state_estimator_msgs iit_commons"
echo "   source install/setup.bash"
echo ""

echo "5. Test ROS 2 node:"
echo "   ros2 launch state_estimator state_estimator.launch.py"
echo ""

echo "6. Start ROS 1 Bridge (if not already running):"
echo "   /root/start_bridge.sh"
echo "   # OR manually:"
echo "   ros2 run ros1_bridge dynamic_bridge --bridge-all-topics"
echo ""

echo "7. Test bridge communication:"
echo "   # Terminal 1 (ROS 1):"
echo "   source /opt/ros/noetic/setup.bash"
echo "   rostopic pub /test_topic std_msgs/String 'data: \"Hello from ROS 1\"'"
echo ""
echo "   # Terminal 2 (ROS 2):"
echo "   source /opt/ros/humble/setup.bash"  
echo "   ros2 topic echo /test_topic"
echo ""

echo "8. Available commands in container:"
echo "   - ros1_ws: ROS 1 workspace with kiss-icp"
echo "   - ros2_ws: ROS 2 workspace with your state estimator"
echo "   - setup_bridge.sh: Source both ROS versions"
echo "   - start_bridge.sh: Launch the bridge"
echo ""

echo "Environment Variables:"
echo "   ROS1_DISTRO=noetic"
echo "   ROS2_DISTRO=humble"
echo "   ROS_MASTER_URI=http://localhost:11311"
echo "   ROS_DOMAIN_ID=0"
echo ""

echo "Troubleshooting:"
echo "- Make sure ROS 1 master is running: roscore"
echo "- Check bridge status: ros2 run ros1_bridge dynamic_bridge --print-pairs"
echo "- View available topics: rostopic list (ROS 1) / ros2 topic list (ROS 2)"
