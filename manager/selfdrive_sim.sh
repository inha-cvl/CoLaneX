#!/bin/bash
cd ../simulator
python3 hlv_simulator.py KIAPI&
python3 rviz_simulator.py hlv KIAPI&
python3 control_ui.py &
cd ../planning
python3 hlv_planning.py KIAPI&
cd ../selfdrive
python3 main.py hlv KIAPI