#! /usr/bin/bash

cd "${1}_ICRA/ICRA_tests"

for d in *
do
	if [ $d -lt 121 ]
	then
		FIRSTFOLDER="${2}_cr_2_pmul"
	elif [ $d -lt 241 ]
	then
		FIRSTFOLDER="${2}_cr_1_pmul"
	elif [ $d -lt 361 ]
	then
		FIRSTFOLDER="${2}_cr_0_pmul"
	fi

	if [ $(($d%4)) -eq 1 ]
        then
                SECONDFOLDER="our"
        elif [ $(($d%4)) -eq 2 ]
	then
                SECONDFOLDER="sheng"
        elif [ $(($d%4)) -eq 3 ]
	then
                SECONDFOLDER="oursheng"
        elif [ $(($d%4)) -eq 0 ]
	then
                SECONDFOLDER="none"
        fi
	
	echo "${d} ~/caf_ws/mission_logs/ICRA_tests_res/${FIRSTFOLDER}/${SECONDFOLDER}"
	mv "${d}" "../../ICRA_tests_res/${FIRSTFOLDER}/${SECONDFOLDER}/not_ordered"	

done

cd ..
cd ..

