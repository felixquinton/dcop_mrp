#!/bin/bash

now=$(date +"%m_%d_%Y_%T")
path="thesis/python/dcop/pydcop"

cd
cd ${path}

python3 mrp_mdvrp_to_yaml.py

pydcop --output metrics/output_${now}.json \
solve -a mgm2 --algo_params stop_cycle:30 \
-d test_mrpp_mdvrp_distribution.yaml -m thread \
--collect_on cycle_change --run_metrics \
metrics/run_time_${now}.csv \
test_mrpp_mdvrp_inst.yaml

\\ python3 res_summary.py \
\\ --input_file=metrics/output_${now}.json \
\\ --output_file=metrics/summary_${now}.csv

cd
