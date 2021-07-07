#! /bin/bash
#
S="source ../../install/setup.bash"
source ../../install/setup.bash

xterm -e bash -c "$S; python3 -m robot_framework.main_kpkdemo_px4 00; bash" &
xterm -e bash -c "$S; python3 -m robot_framework.main_kpkdemo_px4 01; bash" &
xterm -e bash -c "$S; python3 -m robot_framework.main_kpkdemo_px4 02; bash" &
xterm -e bash -c "$S; python3 -m robot_framework.main_kpkdemo_px4 03; bash" &
xterm -e bash -c "$S; python3 -m robot_framework.main_kpkdemo_px4 04; bash" &

../../../start_mission_manager.sh
