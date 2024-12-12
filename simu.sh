gnome-terminal -x bash -c "cd /home/r/Mysoftware/TARE; source devel/setup.bash; roslaunch vehicle_simulator system_garage.launch"
sleep 1
gnome-terminal -x bash -c "cd /home/r/Mysoftware/TARE; source devel/setup.bash; roslaunch tare_planner explore_garage.launch"
sleep 1