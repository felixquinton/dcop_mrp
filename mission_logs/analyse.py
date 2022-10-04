import csv
import pandas as pd
import json
import numpy as np

graph_type = "grid_5x5_r5"

def utf8len(s):
    return len(s.encode('utf-8'))

nb_inst_global = 50
finish_date = 1200
nb_robot = 5
avoid_runs = []
if graph_type == "grid_5x5_r5":
    avoid_runs = []
elif graph_type == "star_5x5_r5":
    avoid_runs = [4, 10, 17, 18, 20] #2, 9, 12
elif graph_type == "caylus_small":
    avoid_runs = [5, 22, 11, 4] #2, 9, 12
elif graph_type == "caylus_large":
    avoid_runs = [1] #2, 9, 12
print(avoid_runs)

finish_max_dict = {method: [] for method in ["ours", "sheng", "no_com"]}
alltime_max_dict = {method: [] for method in ["ours", "sheng", "no_com"]}
nb_msgs_dict = {method: {i: 0 for i in np.delete(np.arange(nb_inst_global), avoid_runs)} for method in ["ours", "sheng", "no_com"]}
size_msgs_dict = {method: {i: 0 for i in np.delete(np.arange(nb_inst_global), avoid_runs)} for method in ["ours", "sheng", "no_com"]}

for auction_type in ["ours", "sheng", "no_com"]:
        first_folder = f"IROS_tests/{graph_type}/{auction_type}"
        for i in range(nb_inst_global):
            if not i in avoid_runs:
                
                # Idleness metrics
                tmp_ind = str(i)
                alltime_max = 0
                folder_address = first_folder + "/" + tmp_ind
                file_address = folder_address + "/idle_inst_5r_20w_r700_01.csv"
                data = pd.read_csv(file_address, header=None)
                total_simulated_time = data[1][len(data[1])-1]
                for j in range(len(data[0])):
                    idleness = json.loads(data[3][j])
                    if data[1][j] > finish_date:
                        offset = data[1][j] - finish_date
                        worst_visit = np.min(list(idleness.values()))
                        end_max_idleness = finish_date - worst_visit + offset
                        finish_max_dict[auction_type].append(end_max_idleness)
                        break
                    else:
                        tmp_max_idleness = np.max(np.ones(len(idleness))*data[1][j]-np.array(list(idleness.values())))
                        if tmp_max_idleness > alltime_max:
                            alltime_max = tmp_max_idleness
                print("----------------")
                print(i, data[1][len(data[1])-1])
                alltime_max_dict[auction_type].append(alltime_max)
                print(auction_type, i, alltime_max)
                print(auction_type, i, end_max_idleness)
                """
                # Message metrics
                folder_address = first_folder + "/" + tmp_ind
                for r in range(1, nb_robot+1):
                    file_address = folder_address + f"/passed_msg_robot_{r}.csv"
                    data = pd.read_csv(file_address, header=None)
                    for j in range(len(data[0])):
                        nb_msgs_dict[auction_type][i] += 1
                        size_msgs_dict[auction_type][i] += utf8len(data[2][j])
                """
print('------------------------------------') 
for i in range(len(alltime_max_dict["no_com"])):
    print(i, alltime_max_dict["no_com"][i] - alltime_max_dict["ours"][i])
print('------------------------------------') 
print(finish_max_dict)
finish_max_ratio = {"oursvsno_com": np.mean((np.array(finish_max_dict["ours"]) - np.array(finish_max_dict["no_com"])) / np.array(finish_max_dict["no_com"])),
                    "oursvssheng": np.mean((np.array(finish_max_dict["ours"]) - np.array(finish_max_dict["sheng"])) / np.array(finish_max_dict["sheng"]))}
alltime_max_ratio = {"oursvsno_com": np.mean((np.array(alltime_max_dict["ours"]) - np.array(alltime_max_dict["no_com"])) / np.array(alltime_max_dict["no_com"])),
                     "oursvssheng": np.mean((np.array(alltime_max_dict["ours"]) - np.array(alltime_max_dict["sheng"])) / np.array(alltime_max_dict["sheng"]))}
nb_msgs_ratio = {"oursvsno_com": np.mean((np.array(list(nb_msgs_dict["ours"].values())) - np.array(list(nb_msgs_dict["no_com"].values()))) / np.array(list(nb_msgs_dict["no_com"].values()))),
                 "oursvssheng": np.mean((np.array(list(nb_msgs_dict["ours"].values())) - np.array(list(nb_msgs_dict["sheng"].values()))) / np.array(list(nb_msgs_dict["sheng"].values())))}
size_msgs_dict = {"oursvsno_com": np.mean((np.array(list(size_msgs_dict["ours"].values())) - np.array(list(size_msgs_dict["no_com"].values()))) / np.array(list(size_msgs_dict["no_com"].values()))),
                 "oursvssheng": np.mean((np.array(list(size_msgs_dict["ours"].values())) - np.array(list(size_msgs_dict["sheng"].values()))) / np.array(list(size_msgs_dict["sheng"].values())))}
print(finish_max_ratio)
print(alltime_max_ratio)
print(nb_msgs_ratio)
print(size_msgs_dict)
