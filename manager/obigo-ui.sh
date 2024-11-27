#!/bin/bash
cd ../ui-obigo
roslaunch rosbridge_server rosbridge_websocket.launch &
./gradlew bootRun &
google-chrome http://localhost:8080 