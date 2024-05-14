#!/bin/bash
#When Use sensors
#cd ../car/src
#python3 main.py &
#cd ../../simulator

#When Use OBU
cd ../car/src
python3 main.py &
cd ../../simulator
python3 rviz_simulator.py tlv Harbor&
python3 simple_ui.py &
cd ../planning
python3 tlv_planning.py Harbor&
cd ../v2x2 
python3 main.py 2