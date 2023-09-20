#!/bin/bash
cd ../car
python3 obu.py tlv
cd ../simulator
rviz -d rviz/field.rviz &
cd ../planning
python3 dynamic_path.py &
python3 map_publisher.py & 
rosrun v2x tlv