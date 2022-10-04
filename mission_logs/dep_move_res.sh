#! /usr/bin/bash

mkdir ICRA_tests/simu_icra_obs_1_cr_100
mkdir ICRA_tests/simu_icra_obs_05_cr_100
mkdir ICRA_tests/simu_icra_obs_0_cr_100
mkdir ICRA_tests/simu_icra_obs_1_cr_50
mkdir ICRA_tests/simu_icra_obs_05_cr_50
mkdir ICRA_tests/simu_icra_obs_0_cr_50
mkdir ICRA_tests/simu_icra_obs_1_cr_70
mkdir ICRA_tests/simu_icra_obs_05_cr_70
mkdir ICRA_tests/simu_icra_obs_0_cr_70

cd ICRA_tests

for i in {1..1350}
do
  if [ $i -lt 51 ]
  then
    mv $i simu_icra_obs_1_cr_50/our
  elif [ $i -lt 101 ]
  then
    mv $i simu_icra_obs_1_cr_50/sheng
  elif [ $i -lt 151 ]
  then
    mv $i simu_icra_obs_1_cr_50/none
  elif [ $i -lt 201 ]
  then
    mv $i simu_icra_obs_05_cr_50/our
  elif [ $i -lt 251 ]
  then
    mv $i simu_icra_obs_05_cr_50/sheng
  elif [ $i -lt 301 ]
  then
    mv $i simu_icra_obs_05_cr_50/none
  elif [ $i -lt 351 ]
  then
    mv $i simu_icra_obs_0_cr_50/our
  elif [ $i -lt 401 ]
  then
    mv $i simu_icra_obs_0_cr_50/sheng
  elif [ $i -lt 451 ]
  then
    mv $i simu_icra_obs_0_cr_50/none
  elif [ $i -lt 501 ]
  then
    mv $i simu_icra_obs_1_cr_70/our
  elif [ $i -lt 551 ]
  then
    mv $i simu_icra_obs_1_cr_70/sheng
  elif [ $i -lt 601 ]
  then
    mv $i simu_icra_obs_1_cr_70/none
  elif [ $i -lt 651 ]
  then
    mv $i simu_icra_obs_05_cr_70/our
  elif [ $i -lt 701 ]
  then
    mv $i simu_icra_obs_05_cr_70/sheng
  elif [ $i -lt 751 ]
  then
    mv $i simu_icra_obs_05_cr_70/none
  elif [ $i -lt 801 ]
  then
    mv $i simu_icra_obs_0_cr_70/our
  elif [ $i -lt 851 ]
  then
    mv $i simu_icra_obs_0_cr_70/sheng
  elif [ $i -lt 901 ]
  then
    mv $i simu_icra_obs_0_cr_70/none
  elif [ $i -lt 951 ]
  then
    mv $i simu_icra_obs_1_cr_100/our
  elif [ $i -lt 1001 ]
  then
    mv $i simu_icra_obs_1_cr_100/sheng
  elif [ $i -lt 1051 ]
  then
    mv $i simu_icra_obs_1_cr_100/none
  elif [ $i -lt 1101 ]
  then
    mv $i simu_icra_obs_05_cr_100/our
  elif [ $i -lt 1151 ]
  then
    mv $i simu_icra_obs_05_cr_100/sheng
  elif [ $i -lt 1201 ]
  then
    mv $i simu_icra_obs_05_cr_100/none
  elif [ $i -lt 1251 ]
  then
    mv $i simu_icra_obs_0_cr_100/our
  elif [ $i -lt 1301 ]
  then
    mv $i simu_icra_obs_0_cr_100/sheng
  elif [ $i -lt 1351 ]
  then
    mv $i simu_icra_obs_0_cr_100/none
  else
    echo "False number."
  fi
done
