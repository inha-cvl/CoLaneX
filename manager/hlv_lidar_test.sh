#!/bin/bash
cd ../car
python3 ioniq5.py &
cd ../simulator
python3 rviz_simulator.py hlv KCity&
python3 control_ui.py &
cd ../planning
python3 hlv_planning.py KCity&
cd ../selfdrive
python3 main.py hlv KCity &
cd ../utils
python3 test_logger.py $1 lidar &