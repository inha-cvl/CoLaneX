#!/bin/bash
cd ../simulator
python3 tlv_simulator.py &
python3 hlv_simulator.py &
python3 fake_v2x.py &
python3 rviz_simulator.py hlv&
python3 simple_ui.py &
cd ../planning
python3 hlv_planning.py &
python3 tlv_planning.py &
cd ../selfdrive
python3 main.py hlv &
python3 main.py tlv 