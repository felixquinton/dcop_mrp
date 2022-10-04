#! /usr/bin/bash

cd ICRA_tests_res

for d in *
do	
	cd $d
	for e in *
	do
		cd $e
		cd not_ordered
		CPT=31 
		for f in *
		do
			echo $d
			echo $e
			echo $f
			echo $CPT
			mv $f "../${CPT}"
			CPT=$(($CPT+1))
		done
		cd ..
		cd ..
	done
	cd ..
done

cd ..
