#!/bin/bash
#When Use sensors
#cd ../car/src
#python3 main.py &
#cd ../../simulator

#When Use OBU
cd ../car
python3 car.py tlv&
cd ../simulator
python3 rviz_simulator.py tlv Solbat&
python3 simple_ui.py &
cd ../planning
python3 tlv_planning.py Solbat&
cd ../v2x2 
python3 main.py 2 0 out