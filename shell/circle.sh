#!/bin/bash
source setup_px4_env.sh
gnome-terminal --window -e 'bash -c "cd ~/airhust_ws/; roslaunch simulation iris_rplidar.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; cd ~/airhust_ws/ ; roslaunch complete_mission implement.launch; exec bash"'\
gnome-terminal --window -e 'bash -c "sleep 5; cd ~/airhust_ws/ ; roslaunch complete_mission circle.launch; exec bash"'
