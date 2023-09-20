#!/bin/bash
cd ../simulator
python3 mode_pub_ui.py &
python3 tlv_simulator.py &
python3 hlv_simulator.py &
python3 fake_v2x.py &
rviz -d rviz/simulator.rviz &
cd ../planning
python3 dynamic_path.py &
python3 safe_distance.py &
cd ../selfdrive
python3 main.py hlv &
python3 main.py tlv 