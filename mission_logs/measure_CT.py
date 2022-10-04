import json
from math import prod

inst_type = "caylus_graphs_large"
path = f"../src/patrol_mission_spec/data/generated_scenarios/{inst_type}"

if inst_type == "caylus_graphs_small":
    nb_scenario = 65
    nb_waypoint = 21
    nb_robots = 5
    types = {"L": 1, "N": 2, "D": 2}
elif inst_type == "caylus_graphs_large":
    nb_scenario = 50
    nb_waypoint = 46
    nb_robots = 10
    types = {"L": 2, "N": 4, "D": 4}

sum_ct = 0
sum_pr = 0
for s_id in range(nb_scenario):
    with open(f"{path}/scenario_{s_id}/data.json") as input_file:
        data = json.loads(json.load(input_file))
    w_ts = {"N": nb_waypoint, "L": nb_waypoint, "D": nb_waypoint}
    r_ws = {w: 5 for w in range(nb_waypoint)}  # Number of robots able to execute each waypoint
    for event in data["node_events"]:
        w_ts["D"] -= 1
        if "night_camera" in event["sensor_types"]:
            r_ws[event['event_node']] = 3
        else:
            w_ts["N"] -= 1
            r_ws[event['event_node']] = 1
    ct = w_ts["L"]/types["L"] * (w_ts["N"]/types["N"]) * (w_ts["D"]/types["D"])
    pr = 1/nb_robots**nb_waypoint*prod(list(r_ws.values()))
    sum_ct += ct
    sum_pr += pr
print(sum_ct/nb_scenario)
print(sum_pr/nb_scenario)
