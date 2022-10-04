#! /usr/bin/bash

NBINSTANCE=49

mkdir mission_logs/PAAMS_auctions
mkdir mission_logs/PAAMS_auctions/grid_graphs_dim=7x7_obsprop=0.15_p60
mkdir mission_logs/PAAMS_auctions/grid_graphs_dim=7x7_obsprop=0.15_p60/sheng
mkdir mission_logs/PAAMS_auctions/grid_graphs_dim=7x7_obsprop=0.15_p60/sheng/backup


for (( NUMBER=0; NUMBER<=$NBINSTANCE; NUMBER++ ))
do
        echo $NUMBER
        timeout 600s ros2 launch patrol_sim_robots inst_caylus_r10_w21_launch.py scenario_path:="generated_scenarios/grid_graphs_dim=(7, 7)_obsprop=0.15/scenario_${NUMBER}/data.json" communication_range:=60 scenario_id:=${NUMBER} auction_com_cost:=sheng
	
	mv mission_logs/PAAMS_auctions/${NUMBER} mission_logs/PAAMS_auctions/grid_graphs_dim=7x7_obsprop=0.15_p60/sheng
	
	ros2 daemon stop
done


for (( NUMBER=0; NUMBER<=$NBINSTANCE; NUMBER++ ))
do
        echo $NUMBER
        timeout 600s ros2 launch patrol_sim_robots inst_caylus_r10_w21_launch.py scenario_path:="generated_scenarios/grid_graphs_dim=(7, 7)_obsprop=0.15/scenario_${NUMBER}/data.json" communication_range:=60 scenario_id:=${NUMBER} auction_com_cost:=sheng
	
	mv mission_logs/PAAMS_auctions/${NUMBER} mission_logs/PAAMS_auctions/grid_graphs_dim=7x7_obsprop=0.15_p60/sheng/backup
	
	ros2 daemon stop
done

mkdir mission_logs/PAAMS_auctions
mkdir mission_logs/PAAMS_auctions/grid_graphs_dim=7x7_obsprop=0.15_p75
mkdir mission_logs/PAAMS_auctions/grid_graphs_dim=7x7_obsprop=0.15_p75/sheng
mkdir mission_logs/PAAMS_auctions/grid_graphs_dim=7x7_obsprop=0.15_p75/sheng/backup


for (( NUMBER=0; NUMBER<=$NBINSTANCE; NUMBER++ ))
do
        echo $NUMBER
        timeout 600s ros2 launch patrol_sim_robots inst_caylus_r10_w21_launch.py scenario_path:="generated_scenarios/grid_graphs_dim=(7, 7)_obsprop=0.15/scenario_${NUMBER}/data.json" communication_range:=75 scenario_id:=${NUMBER} auction_com_cost:=sheng
	
	mv mission_logs/PAAMS_auctions/${NUMBER} mission_logs/PAAMS_auctions/grid_graphs_dim=7x7_obsprop=0.15_p75/sheng
	
	ros2 daemon stop
done


for (( NUMBER=0; NUMBER<=$NBINSTANCE; NUMBER++ ))
do
        echo $NUMBER
        timeout 600s ros2 launch patrol_sim_robots inst_caylus_r10_w21_launch.py scenario_path:="generated_scenarios/grid_graphs_dim=(7, 7)_obsprop=0.15/scenario_${NUMBER}/data.json" communication_range:=75 scenario_id:=${NUMBER} auction_com_cost:=sheng
	
	mv mission_logs/PAAMS_auctions/${NUMBER} mission_logs/PAAMS_auctions/grid_graphs_dim=7x7_obsprop=0.15_p75/sheng/backup
	
	ros2 daemon stop
done

mkdir mission_logs/PAAMS_auctions
mkdir mission_logs/PAAMS_auctions/grid_graphs_dim=7x7_obsprop=0.15_p100
mkdir mission_logs/PAAMS_auctions/grid_graphs_dim=7x7_obsprop=0.15_p100/sheng
mkdir mission_logs/PAAMS_auctions/grid_graphs_dim=7x7_obsprop=0.15_p100/sheng/backup


for (( NUMBER=0; NUMBER<=$NBINSTANCE; NUMBER++ ))
do
        echo $NUMBER
        timeout 600s ros2 launch patrol_sim_robots inst_caylus_r10_w21_launch.py scenario_path:="generated_scenarios/grid_graphs_dim=(7, 7)_obsprop=0.15/scenario_${NUMBER}/data.json" communication_range:=100 scenario_id:=${NUMBER} auction_com_cost:=sheng
	
	mv mission_logs/PAAMS_auctions/${NUMBER} mission_logs/PAAMS_auctions/grid_graphs_dim=7x7_obsprop=0.15_p100/sheng
	
	ros2 daemon stop
done


for (( NUMBER=0; NUMBER<=$NBINSTANCE; NUMBER++ ))
do
        echo $NUMBER
        timeout 600s ros2 launch patrol_sim_robots inst_caylus_r10_w21_launch.py scenario_path:="generated_scenarios/grid_graphs_dim=(7, 7)_obsprop=0.15/scenario_${NUMBER}/data.json" communication_range:=100 scenario_id:=${NUMBER} auction_com_cost:=sheng
	
	mv mission_logs/PAAMS_auctions/${NUMBER} mission_logs/PAAMS_auctions/grid_graphs_dim=7x7_obsprop=0.15_p100/sheng/backup
	
	ros2 daemon stop
done

mkdir mission_logs/PAAMS_auctions
mkdir mission_logs/PAAMS_auctions/grid_graphs_dim=7x7_obsprop=0.15_p200
mkdir mission_logs/PAAMS_auctions/grid_graphs_dim=7x7_obsprop=0.15_p200/sheng
mkdir mission_logs/PAAMS_auctions/grid_graphs_dim=7x7_obsprop=0.15_p200/sheng/backup

for (( NUMBER=0; NUMBER<=$NBINSTANCE; NUMBER++ ))
do
        echo $NUMBER
        timeout 600s ros2 launch patrol_sim_robots inst_caylus_r10_w21_launch.py scenario_path:="generated_scenarios/grid_graphs_dim=(7, 7)_obsprop=0.15/scenario_${NUMBER}/data.json" communication_range:=200 scenario_id:=${NUMBER} auction_com_cost:=sheng
	
	mv mission_logs/PAAMS_auctions/${NUMBER} mission_logs/PAAMS_auctions/grid_graphs_dim=7x7_obsprop=0.15_p200/sheng
	
	ros2 daemon stop
done

for (( NUMBER=0; NUMBER<=$NBINSTANCE; NUMBER++ ))
do
        echo $NUMBER
        timeout 600s ros2 launch patrol_sim_robots inst_caylus_r10_w21_launch.py scenario_path:="generated_scenarios/grid_graphs_dim=(7, 7)_obsprop=0.15/scenario_${NUMBER}/data.json" communication_range:=200 scenario_id:=${NUMBER} auction_com_cost:=sheng
	
	mv mission_logs/PAAMS_auctions/${NUMBER} mission_logs/PAAMS_auctions/grid_graphs_dim=7x7_obsprop=0.15_p200/sheng/backup
	
	ros2 daemon stop
done

