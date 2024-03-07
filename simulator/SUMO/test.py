import os
import sys
if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))

from sumolib import checkBinary
sumoBinary = checkBinary('sumo-gui')
sumoCmd = [sumoBinary, "-c", "./right35.sumocfg"]

import traci
import traci.constants as tc

traci.start(sumoCmd)
traci.vehicle.subscribe("v_0", (tc.VAR_ROAD_ID, tc.VAR_LANEPOSITION))


for step in range(100):
    traci.simulationStep()
    v0_speed = traci.vehicle.getSpeed("v_0")
    v1_speed = traci.vehicle.getSpeed("v_1")
traci.close()
