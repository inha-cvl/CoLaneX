#!/bin/bash
cd ../simulator
python3 tlv_simulator.py songdo&
python3 hlv_simulator.py songdo&
python3 fake_v2x.py &
python3 rviz_simulator.py hlv songdo&
# python3 rviz_simulator.py tlv songdo&
python3 simple_ui.py &
cd ../planning
python3 hlv_planning.py songdo&
python3 tlv_planning.py songdo&
cd ../selfdrive
python3 main.py hlv songdo&
python3 main.py tlv songdo2&
cd ../utils
python3 fake_json.py