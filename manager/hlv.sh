#!/bin/bash
# When use sensors
cd ../car
python3 ioniq5.py &
cd ../simulator
python3 rviz_simulator.py hlv Harbor&
python3 simple_ui.py &
cd ../planning
python3 hlv_planning.py Harbor&
cd ../v2x2 
python3 main.py 1