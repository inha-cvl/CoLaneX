#!/bin/bash
cd ../car/src
python3 ioniq5.py &
python3 transceiver.py &
cd ../../simulator
python3 rviz_simulator.py hlv &
python3 simple_ui.py &
cd ../planning
python3 hlv_planning.py &
cd ../selfdrive
python3 main.py hlv &
