# gnome-terminal -x bash -c "cd /home/r/Mysoftware/CarSpace; source devel/setup.bash; sh start.sh"
# sleep 3
# gnome-terminal -x bash -c "cd /home/r/Mysoftware/LIO-SAM; source devel/setup.bash; sh lio_dev.sh"
# sleep 3
gnome-terminal -x bash -c "cd /home/r/Mysoftware/TARE; source devel/setup.bash; rosrun control_handle control_node"
sleep 1
gnome-terminal -x bash -c "cd /home/r/Mysoftware/CarSpace; source devel/setup.bash; rosrun joy joy_node"
sleep 1
gnome-terminal -x bash -c "cd /home/r/Mysoftware/Sensor; source devel/setup.bash; roslaunch gnw_500 gnw_ahrs.launch;read"
sleep 1
gnome-terminal -x bash -c "cd /home/r/Mysoftware/Sensor; source devel/setup.bash; rosrun rslidar_sdk rslidar_sdk_node;read"
sleep 1
gnome-terminal -x bash -c "cd /home/r/Mysoftware/Sensor; source devel/setup.bash; rosrun lidarconv lidarconv_node;read"
sleep 1
gnome-terminal -x bash -c "cd /home/r/Mysoftware/LIO-SAM; source devel/setup.bash; roslaunch lio_sam run.launch;read"
sleep 4
gnome-terminal -x bash -c "cd /home/r/Mysoftware/TARE; source devel/setup.bash; roslaunch vehicle_simulator system_real_robot.launch"
sleep 1
