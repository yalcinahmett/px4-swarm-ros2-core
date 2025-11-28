#!/bin/bash

echo "Starting MicroXRCEAgent..."
terminator --new-tab -e "bash -c 'MicroXRCEAgent udp4 -p 8888; exec bash'" &
sleep 2

# --- 2. QGROUNDCONTROL ---
echo "Starting QGroundControl..."
terminator --new-tab -e "bash -c 'cd ~ && ./QGroundControl*.AppImage; exec bash'" &
sleep 5 

echo "Starting PX4 Instances ..."
terminator --new-tab -e "bash -c 'cd ~/PX4-Autopilot && ./Tools/simulation/gz_multi_vehicle.sh 3 1.5 ; exec bash'" &
sleep 5 
