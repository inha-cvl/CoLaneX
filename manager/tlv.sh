#!/bin/bash
cd ../car
python3 obu.py tlv &
cd ../simulator
python3 rviz_simulator.py tlv &
cd ../planning
python3 tlv_planning.py &
rosrun v2x tlv