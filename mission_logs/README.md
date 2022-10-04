# Run simulations

Script run_simulation.sh runs ROS2 simulations for the MRS surveillance missions.

## Usage

From the command line, call 
```bash
. run_simulation.sh
```

The script has options allowing to pass the scenarios' parameters. *Compulsory* parameters are:
- folder: Path to the folder in which the simulations will be logged.
- nb_inst: Number of scenarios to be run.
- bid_val: The valuation to use in the auction scheme.

*Optional* parameters take default values if not specified:
- timeout: Maximum running time for each scenario's. Default value is 600.
- launch_file: ROS2 launch file that initiates the ROS2 nodes. Must be a launch file from the patrol_sim_robot package, properly installed is the install/patrol_sim_robot/share directory. Default is "inst_caylus_r10_w21_launch.py". 
- scenario_type: Type of scenario to be simulated. Default is grid_graphs_dim=7x7_obsprop=0.15.
- range: robots' communication range. Default value is 100.

## Example

The following runs 50 scenarios with a 5 robots team on the small Caylus map for 600 seconds each, solving MRTA with auctions using *our* valuation formula.
```bash
. mission_logs/run_simulation.sh folder="PATROL_tests" bid_val="ours" nb_inst=50 timeout=600 scenario_type="caylus_graphs_small" launch_file="inst_caylus_r5_w21_launch.py range=100"

```

The following runs 50 scenarios with a 10 robots team on the (7x7) grid maps presented in the PAAMS paper, for 900 seconds each, solving MRTA with auctions using *Sheng's* valuation formula.
```bash
. mission_logs/run_simulation.sh folder="PATROL_tests" bid_val="sheng" nb_inst=50 timeout=900 scenario_type="grid_graphs_dim=(7, 7)_obsprop=0.15" launch_file="inst_caylus_r10_w21_launch.py" range=100 method="SI"

```

## Note

Default value for a given optional parameter does not work if a value was specified before for this parameter. Possible cause is that values are stored into bin/...
Hence, it is advise to always specify optional parameters' values, even if the desired values are the same as the default.
