#!/bin/bash

# Check for 3 input arguments: x, y, and yaw
if [ $# -ne 3 ]; then
  echo "Usage: $0 x y yaw_in_degrees"
  exit 1
fi

# Parse input arguments
x=$1
y=$2
yaw_deg=$3

# Convert yaw from degrees to quaternion using Python
qzqw=$(python3 -c "import math; yaw = math.radians($yaw_deg); print(f'{math.sin(yaw / 2)} {math.cos(yaw / 2)}')")
qz=$(echo $qzqw | cut -d' ' -f1)
qw=$(echo $qzqw | cut -d' ' -f2)

# Convert to float for publishing
qz_float=$(printf "%.6f" $qz)
qw_float=$(printf "%.6f" $qw)

# Stop the TurtleBot by publishing zero velocity to the /cmd_vel topic
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -1

# Publish the reset pose with x, y, and yaw converted to quaternion
rostopic pub /reset_pose geometry_msgs/PoseStamped "header:
  stamp: {secs: 0, nsecs: 0}
  frame_id: 'map'
pose:
  position:
    x: $x
    y: $y
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: $qz_float
    w: $qw_float"
