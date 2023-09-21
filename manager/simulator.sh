#!/bin/bash
cd ../simulator
python3 tlv_simulator.py &
python3 hlv_simulator.py &
python3 fake_v2x.py &
python3 simple_ui.py &
cd ../planning
python3 dynamic_path.py &
python3 safe_distance.py &
cd ../selfdrive
python3 main.py hlv &
python3 main.py tlv 