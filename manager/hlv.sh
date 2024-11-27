#!/bin/bash
# When use sensors
cd ../car
python3 car.py hlv&
cd ../simulator
python3 rviz_simulator.py hlv Solbat&
python3 simple_ui.py &
cd ../planning
python3 hlv_planning.py Solbat&
cd ../v2x2 
python3 main.py 1 0 out