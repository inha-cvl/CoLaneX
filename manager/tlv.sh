#!/bin/bash
#When Use sensors
#cd ../car/src
#python3 main.py &
#cd ../../simulator

#When Use OBU
cd ../car/src
python3 main.py &
cd ../../simulator
python3 rviz_simulator.py tlv Pangyo&
python3 simple_ui.py &
cd ../planning
python3 tlv_planning.py Pangyo&
cd ../v2x2 
python3 main.py 2