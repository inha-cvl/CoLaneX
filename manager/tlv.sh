#!/bin/bash
#cd ../car/src
#python3 main.py &
#cd ../../simulator
cd ../car
python3 obu.py tlv &
cd ../simulator

python3 rviz_simulator.py tlv &
python3 simple_ui.py &
cd ../planning
python3 tlv_planning.py &
cd ../utils
python3 rate_by_dist.py tlv &
rosrun v2x tlv