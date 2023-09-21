#!/bin/bash
cd ../car
python3 obu.py hlv &
cd ../simulator
python3 rviz_simulator hlv &
python3 simple_ui.py &
cd ../planning
python3 hlv_planning.py&
rosrun v2x hlv