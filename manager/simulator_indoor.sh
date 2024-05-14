#!/bin/bash
cd ../simulator
python3 tlv_simulator.py Harbor&
python3 hlv_simulator.py Harbor&
python3 rviz_simulator.py hlv Harbor&
python3 fake_v2x.py &
# python3 rviz_simulator.py tlv Harbor&
python3 simple_ui.py &
cd ../planning
python3 hlv_planning.py Harbor &
python3 tlv_planning.py Harbor