#!/bin/bash
python_script="/home/sara/myagv_ros2/src/myagv_ros2_ydlidar/myagv_ros2_ydlidar/close_ydlidar.py"

if [ -f "$python_script" ];then
   sudo python3 "$python_script"
else
    echo "Python file does not exist"
fi












