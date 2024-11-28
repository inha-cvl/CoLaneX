#!/bin/bash
cd ../simulator
python3 hlv_simulator.py Solbat&
python3 tlv_simulator.py Solbat&
python3 rviz_simulator.py hlv Solbat&
python3 simple_ui.py &
python3 fake_v2x.py&
cd ../planning
python3 hlv_planning.py Solbat&
python3 tlv_planning.py Solbat