#! /usr/bin/bash

for NUMBER in {1..360}
do
        
	FILE="mission_logs/ind.txt"
	if [ -f "$FILE" ]
	then
	  rm $FILE
	else
	  echo "$FILE is not a file."
	fi

	touch $FILE
	echo -n $NUMBER >> $FILE
	

	CR_FILE="mission_logs/com_range.txt"
	if [ -f "$CR_FILE" ]
	then
	  rm $CR_FILE
	else
	  echo "$CR_FILE is not a file."
	fi

	touch $CR_FILE
	if [ $NUMBER -eq 1 ]
	then
		echo -n 100 >> $CR_FILE

	        python3 mission_logs/mod_specs.py

	        colcon build --symlink-install
	        
	        . install/setup.bash
	elif [ $NUMBER -eq 401 ]
	then
		echo -n 75 >> $CR_FILE

	        python3 mission_logs/mod_specs.py

	        colcon build --symlink-install
	        
	        . install/setup.bash
	elif [ $NUMBER -eq 901 ]
	then
		echo -n 100 >> $CR_FILE

	        python3 mission_logs/mod_specs.py

	        colcon build --symlink-install
	        
	        . install/setup.bash
	fi
	
	timeout 600s ros2 launch patrol_sim_robots inst_caylus_r5_w21_launch.py

	sleep 1
	
	ros2 daemon stop

	sleep 3
done
