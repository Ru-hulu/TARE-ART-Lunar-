gnome-terminal -x bash -c "cd /home/r/Mysoftware/Sensor; source devel/setup.bash; roslaunch gnw_500 gnw_ahrs.launch;read"
sleep 1
gnome-terminal -x bash -c "cd /home/r/Mysoftware/Sensor; source devel/setup.bash; rosrun rslidar_sdk rslidar_sdk_node;read"
sleep 1
gnome-terminal -x bash -c "cd /home/r/Mysoftware/Sensor; source devel/setup.bash; rosrun lidarconv lidarconv_node;read"
sleep 1
gnome-terminal -x bash -c "cd /home/r/Mysoftware/LIO-SAM; source devel/setup.bash; roslaunch lio_sam run.launch;read"
sleep 4
# gnome-terminal -x bash -c "rosbag record /lio_sam/mapping/odometry /terrain_map -O /media/r/20004c35-a8af-4f32-bb40-3c2047d7d01d/r/F1211.bag"
gnome-terminal -x bash -c "cd /home/r/Mysoftware/TARE; source devel/setup.bash; rosrun control_handle control_node"
sleep 1
gnome-terminal -x bash -c "cd /home/r/Mysoftware/TARE; source devel/setup.bash; roslaunch vehicle_simulator system_real_robot.launch"
sleep 1
# gnome-terminal -x bash -c "cd /home/r/Mysoftware/TARE; source devel/setup.bash; roslaunch tare_planner explore_garage.launch"
