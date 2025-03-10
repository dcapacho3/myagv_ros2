#!/bin/bash
sudo chmod a+rw /dev/ttyAMA0
python_script="/home/sara/myagv_ros2/src/myagv_ros2_ydlidar/myagv_ros2_ydlidar/start_ydlidar.py"

if [ -f "$python_script" ];then
   sudo python3 "$python_script"
else
    echo "Python file does not exist"
fi
