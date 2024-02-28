#!/bin/bash
#When Use sensors
#cd ../car/src
#python3 main.py &
#cd ../../simulator

#When Use OBU
cd ../car
python3 obu.py tlv &
cd ../simulator
python3 rviz_simulator.py tlv KCity&
python3 simple_ui.py &
cd ../planning
python3 tlv_planning.py KCity&
cd ../utils
python3 points_publisher.py &

rosrun v2x tlv