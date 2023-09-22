#!/bin/bash
cd ../car/src
python3  main.py &
cd ../../simulator
python3 rviz_simulator.py hlv &
python3 simple_ui.py &
cd ../planning
python3 hlv_planning.py &
rosrun v2x hlv