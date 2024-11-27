#!/bin/bash
cd ../simulator
python3 hlv_simulator.py Pangyo&
python3 tlv_simulator.py Pangyo&
python3 rviz_simulator.py hlv Pangyo&
python3 simple_ui.py &
python3 fake_v2x.py&
cd ../planning
python3 hlv_planning.py Pangyo&
python3 tlv_planning.py Pangyo&
cd ../selfdrive
python3 main.py hlv Pangyo &
python3 main.py tlv Pangyo 