#! /usr/bin/bash

if [ ! -d "ICRA_tests_res" ]
then
	mkdir ICRA_tests_res
fi

cd ICRA_tests_res

for PMUL in 0 1 2
do
	for CR in 40 50 60 75
	do
		if [ ! -d "${CR}_cr_${PMUL}_pmul" ]
		then
			mkdir "${CR}_cr_${PMUL}_pmul"
		fi

		cd "${CR}_cr_${PMUL}_pmul"

		for AUC_TYPE in "our" "sheng" "oursheng" "none"
		do
	                if [ ! -d $AUC_TYPE ]
	                then
        	                mkdir $AUC_TYPE
                	fi

			cd $AUC_TYPE

			if [ ! -d "not_ordered" ]
                        then
                                mkdir not_ordered
                        fi

			cd ..

		done
		cd ..
	done
done

cd ..
