#!/bin/bash
cd ../simulator
python3 hlv_simulator.py Harbor&
python3 rviz_simulator.py hlv Harbor&
python3 control_ui.py &
cd ../planning
python3 hlv_planning.py Harbor&
cd ../selfdrive
python3 main.py hlv Harbor &
cd ../utils
python3 points_publisher.py 