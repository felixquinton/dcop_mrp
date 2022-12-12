#! /usr/bin/bash

for ARGUMENT in "$@"
do
   KEY=$(echo $ARGUMENT | cut -f1 -d=)

   KEY_LENGTH=${#KEY}
   VALUE="${ARGUMENT:$KEY_LENGTH+1}"
   printf -v "${KEY}" "%s" "${VALUE}"
done

VarArray=( "folder" "nb_inst" "method" )

for VAR in ${VarArray[*]}; do
        if [ -z ${!VAR} ];
        then
                echo "E: '${VAR}' option undefined."
                return 0
        fi
done

declare -A ParamHashArray=( ["bid_val"]="ours"
                            ["timeout"]=600
                            ["launch_file"]=inst_caylus_r10_w21_launch.py
                            ["scenario_type"]=grid_graphs_dim=7x7_obsprop=0.15
                            ["range"]=100 )

for PARAM in ${!ParamHashArray[@]}; do
        echo ${!PARAM}
        if [ -z ${!PARAM} ];
        then
                echo "W: '${PARAM}' parameter unspecified. Using default value: ${ParamHashArray[$PARAM]}."
        else
                ParamHashArray[$PARAM]=${!PARAM}
        fi
done


RangeArray=( 75 100 60 )

for RANGE in ${RangeArray[*]};
do

        mkdir ${folder}
        mkdir ${folder}/${ParamHashArray["scenario_type"]}_p$RANGE
        mkdir ${folder}/${ParamHashArray["scenario_type"]}_p$RANGE/${bid_val}
        
        for (( NUMBER=1; NUMBER<=$nb_inst; NUMBER++ ))
        do
                echo $NUMBER
                timeout ${ParamHashArray["timeout"]}s ros2 launch patrol_sim_robots ${ParamHashArray["launch_file"]} scenario_path:="generated_scenarios/${ParamHashArray["scenario_type"]}/scenario_${NUMBER}/data.json" communication_range:=$RANGE scenario_id:=${NUMBER} method:=${method} auction_com_cost:=${ParamHashArray["bid_val"]} folder:=${folder}
	        
	        mv ${folder}/${NUMBER} ${folder}/${ParamHashArray["scenario_type"]}_p$RANGE/${bid_val}
	        
	        ros2 daemon stop
        done
done
