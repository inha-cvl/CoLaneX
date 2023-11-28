#!/bin/bash
cd ../car
python3 morai_car.py&
cd ../simulator
python3 rviz_simulator.py hlv&
python3 simple_ui.py &
cd ../planning
python3 hlv_planning.py &
cd ../selfdrive
python3 main.py hlv &
python3 morai.py 