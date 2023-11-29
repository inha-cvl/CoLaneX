#!/bin/bash
cd ../car
python3 morai.py&
cd ../simulator
python3 rviz_simulator.py hlv&
python3 control_ui.py &
cd ../planning
python3 hlv_planning.py &
cd ../selfdrive
python3 main.py hlv 