#! /usr/bin/bash

cd ICRA_tests_res/simu_icra_obs_0_cr_50/sheng




for g in *; do
  echo $g
  re='^[0-9]+$'
  if ! [[ $yournumber =~ $re ]] ; then
    NUMBER=$(($g%50))
    echo $NUMBER
    mv $g $NUMBER
  fi
done
mv 0 50
cd ../

cd ..
cd ..

