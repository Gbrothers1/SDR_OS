#!/bin/bash
# Start ROS2 sensor telemetry for Robot Controller

# Check if ROS2 is sourced
if [[ -z "$ROS_DISTRO" ]]; then
    echo "ROS2 environment not sourced. Attempting to source it..."
    if [ -f /opt/ros/jazzy/setup.bash ]; then
        source /opt/ros/jazzy/setup.bash
    elif [ -f /opt/ros/foxy/setup.bash ]; then
        source /opt/ros/foxy/setup.bash
    else
        echo "ERROR: Could not find ROS2 setup.bash. Please source ROS2 manually."
        exit 1
    fi
fi

echo "Starting ROS environment for robot telemetry..."

# Start rosbridge in a separate terminal
echo "Starting rosbridge_server..."
gnome-terminal --tab -- bash -c "ros2 launch rosbridge_server rosbridge_websocket_launch.xml; exec bash" &
sleep 2

# Run the IIO telemetry publisher
echo "Starting IIO Telemetry Publisher..."
python3 iio_telemetry_publisher.py --rate 20

# Note: The script will keep running until the telemetry publisher is stopped
# To stop everything, press Ctrl+C in this terminal 
