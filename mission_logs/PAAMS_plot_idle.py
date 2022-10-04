import csv
import pandas as pd
import json
import numpy as np
from matplotlib import pyplot as plt

root_folder = "PAAMS_auctions"
instance_dir = "caylus_graphs_large_events_from_1200_to_1500_p200" #"caylus_graphs_large_events_from_1200_to_1500_p100"caylus_small_r5_p200 caylus_large_r10_p75grid_graphs_dim_7_events_from_1200_to_1500_pstar_graphs_nb_branch=10_branch_length=5_obsprop=0.15_p62

pmul="none"

nb_inst = 50

speed = 1

tota_data_com = {}
tota_data_max_com = {}
tota_mean_com_i1_k1 = {}
tota_max_com_i1_k1 = {}

colors = {"ours": 'b', "sheng": 'r', "no_com": "g", "our+sheng": "m"}
colors_per_pmul = {
    "ours": {"1": "b", "0": "cyan", "none": "mediumpurple"},
    "sheng": {"1": "r", "0": "orangered", "none": "lightcoral"},
    "no_com": {"1": "g", "0": "lime", "none": "darkgreen"}}
reg_colors = {"ours": "c", "sheng": 'orangered', "no_com": "lime",
              "our+sheng": "pink"}

auction_types = ["ours", "no_com", "sheng"]  # ]  # 

inst_to_avoid = []
if instance_dir == "caylus_graphs_large_events_from_900_to_1200":
        inst_to_avoid = [2, 3, 6]
elif instance_dir == "caylus_graphs_large_events_from_1200_to_1500_p60":
        inst_to_avoid = []
elif instance_dir == "caylus_graphs_large_events_from_1200_to_1500_p75":
        inst_to_avoid = []
elif instance_dir == "caylus_graphs_large_events_from_1200_to_1500_p100":
        inst_to_avoid = []
elif instance_dir == "caylus_graphs_large_events_from_1200_to_1500_p200":
        inst_to_avoid = []
elif instance_dir == "grid_graphs_dim_7_events_from_1200_to_1500_p60":
        inst_to_avoid = []
elif instance_dir == "grid_graphs_dim_7_events_from_1200_to_1500_p75":
        inst_to_avoid = []
elif instance_dir == "grid_graphs_dim_7_events_from_1200_to_1500_p100":
        inst_to_avoid = []
elif instance_dir == "grid_graphs_dim_7_events_from_1200_to_1500_p200":
        inst_to_avoid = []
elif instance_dir == "star_graphs_nb_branch=10_branch_length=5_obsprop=0.15_p62":
        inst_to_avoid = [2, 5, 9, 11, 12, 15, 16, 20, 21, 23, 26, 29, 36, 37, 42, 46]
        
nb_inst_dic = {"our+sheng": 15, "ours": nb_inst-len(inst_to_avoid), "sheng": nb_inst-len(inst_to_avoid), "no_com": nb_inst-len(inst_to_avoid)}
        
nb_waypoints = 0
if "star" in instance_dir:
        nb_waypoints = 51
elif "large" in instance_dir:
        nb_waypoints = 46
elif instance_dir == "grid_10x10_r10":
        nb_waypoints = 100
elif instance_dir == "grid_5x5_r5":
    nb_waypoints = 25
elif instance_dir == "star_5x5_r5":
    nb_waypoints = 26
elif "caylus_small_r5" in instance_dir :
    nb_waypoints = 21
elif "dim_7" in instance_dir :
    nb_waypoints = 49
        
print(f"Analyse de {instance_dir} avec {auction_types}")

cpt = 0

for cr in [300]:
        tota_data_com[cr] = {}
        tota_data_max_com[cr] = {}
        tota_mean_com_i1_k1[cr] = {}
        tota_max_com_i1_k1[cr] = {}
        end_ind_list = []
        for auction_type in auction_types:
                print(f"Starting instances with com {auction_type} term")
                first_folder = f"{root_folder}/{instance_dir}/{auction_type}"
                end_ind_com = 3600
                tota_data_com[cr][auction_type] = {i: [] for i in range(end_ind_com)}
                tota_data_max_com[cr][auction_type] = {i: [] for i in range(end_ind_com)}
                tota_mean_com_i1_k1[cr][auction_type] = {i: 0 for i in range(end_ind_com)}
                tota_max_com_i1_k1[cr][auction_type] = {i: 0 for i in range(end_ind_com)}
                for i in range(nb_inst):
                    if i in inst_to_avoid: continue
                    cpt += 1
                    tmp_ind = str(i)
                    folder_address = first_folder + "/" + tmp_ind
                    folder_address = first_folder + "/" + tmp_ind
                    file_address = folder_address + "/idleness_data.csv"
                    data = pd.read_csv(file_address, header=None)
                    first_line = data.iloc[[0]]
                    first_dic = json.loads(first_line[3][0])
                    first_time = round(first_line[1][0])
                    data_dic = {}
                    data_dic[0.0] = {str(w): 1 for w in range(nb_waypoints)}
                    data_mean = {}
                    data_max = {}
                    previous_time = 0.0
                    for ind, line in enumerate(data.iterrows()):
                            if round(line[1][1]) < 3600:
                                new_time = round(line[1][1])
                                read_json = json.loads(line[1][3])
                                if new_time-previous_time > 1:
                                    adding_cpt = 1
                                    for t in range(int(previous_time+1), int(new_time)):
                                        data_dic[float(t)] = {w: data_dic[previous_time][w] + adding_cpt
				                                       for w in data_dic[previous_time]}
                                        data_mean[float(t)] = np.mean(list(data_dic[float(t)].values()))
                                        data_max[float(t)] = np.max(list(data_dic[float(t)].values()))
                                        adding_cpt += 1
                                data_dic[new_time] = {w: data_dic[max(new_time-1, 0)][w] + 1 for w in data_dic[previous_time]}
                                for w in read_json:
                                    if not w == "mean" and not w == "max":
                                        if new_time - read_json[w] < data_dic[new_time][w]:
                                            data_dic[new_time][w] = new_time - read_json[w]
                                data_mean[new_time] = np.mean(list(data_dic[new_time].values()))
                                data_max[new_time] = np.max(list(data_dic[new_time].values()))
                                previous_time = new_time
                    if previous_time < end_ind_com:
                        end_ind_com = previous_time
                    for t in data_mean:
                        tota_mean_com_i1_k1[cr][auction_type][t] += data_mean[t]
                    for t in data_max:
                        tota_max_com_i1_k1[cr][auction_type][t] += data_max[t]
                    for t in data_mean:
                        tota_data_com[cr][auction_type][t].append(data_mean[t])
                    for t in data_max:
                        tota_data_max_com[cr][auction_type][t].append(data_max[t])
                    print(i, previous_time, data_max[t])
                    plt.plot(list(data_mean.keys()), list(data_mean.values()), linewidth=.25, label=f"Range={cr}, com, mean")
                    plt.legend()
                    plt.savefig(folder_address + "/mean_idleness.pdf")
                    plt.clf()
                    plt.plot(list(data_max.keys()), list(data_max.values()), linewidth=.25, label=f"Range={cr}, com, max")
                    plt.legend()
                    plt.savefig(folder_address + "/max_idleness.pdf")
                    plt.clf()
                end_ind_com = int(end_ind_com)
                print(end_ind_com)
                end_ind_list.append(end_ind_com)
                print("COpteur", cpt)

        end_ind_com = min(end_ind_list)
        for auction_type in auction_types:
            nb_inst = nb_inst_dic[auction_type]
            plt.plot(list(tota_mean_com_i1_k1[cr][auction_type].keys())[:end_ind_com],
                     np.array(list(tota_mean_com_i1_k1[cr][auction_type].values()))[:end_ind_com]/nb_inst,
                     label=f"Range={cr}, com={auction_type}, mean", color=colors[auction_type])


            # sigma
            std_array = []
            for v in tota_data_com[cr][auction_type].values():
                std_array.append(np.std(list(v)))
            plt.fill_between(np.array(list(tota_mean_com_i1_k1[cr][auction_type].keys())[:end_ind_com]),
                             np.array(list(tota_mean_com_i1_k1[cr][auction_type].values()))[:end_ind_com]/nb_inst+np.array(std_array)[:end_ind_com],
                             np.array(list(tota_mean_com_i1_k1[cr][auction_type].values()))[:end_ind_com]/nb_inst-np.array(std_array)[:end_ind_com],
                             linewidth=1, alpha=.1, color=colors[auction_type])
            plt.plot(np.array(list(tota_mean_com_i1_k1[cr][auction_type].keys())[:end_ind_com]),
                     np.array(list(tota_mean_com_i1_k1[cr][auction_type].values()))[:end_ind_com]/nb_inst+np.array(std_array)[:end_ind_com],
                     linewidth=1, alpha=.2, color=colors[auction_type])
            plt.plot(np.array(list(tota_mean_com_i1_k1[cr][auction_type].keys())[:end_ind_com]),
                     np.array(list(tota_mean_com_i1_k1[cr][auction_type].values()))[:end_ind_com]/nb_inst-np.array(std_array)[:end_ind_com],
                     linewidth=1, alpha=.2, color=colors[auction_type])
            plt.plot(np.array(list(tota_mean_com_i1_k1[cr][auction_type].keys())[:end_ind_com]),
                     np.array(list(tota_mean_com_i1_k1[cr][auction_type].values()))[:end_ind_com]/nb_inst,
                     linewidth=1, alpha=1, linestyle="dashed", color=colors[auction_type])


        plt.legend()
        plt.savefig(f"{root_folder}/{instance_dir}/mean_idleness_cr_{cr}.pdf")
        plt.clf()
        d_name = {"ours": "I + K", "sheng": "I + S", "no_com": "I"}
        font = {'family': 'serif', 'weight': 'normal', 'size': 16}

        for auction_type in auction_types:
            nb_inst = nb_inst_dic[auction_type]
            plt.plot(list(tota_max_com_i1_k1[cr][auction_type].keys())[:end_ind_com],
                     np.array(list(tota_max_com_i1_k1[cr][auction_type].values()))[:end_ind_com]/nb_inst,
                     label=f"B = {d_name[auction_type]}", color=colors[auction_type])


            # sigma
            std_array = []
            for v in tota_data_com[cr][auction_type].values():
                std_array.append(np.std(list(v)))
            plt.fill_between(np.array(list(tota_max_com_i1_k1[cr][auction_type].keys())[:end_ind_com]),
                             np.array(list(tota_max_com_i1_k1[cr][auction_type].values()))[:end_ind_com]/nb_inst+np.array(std_array)[:end_ind_com]/nb_inst**0.5*2,
                             np.array(list(tota_max_com_i1_k1[cr][auction_type].values()))[:end_ind_com]/nb_inst-np.array(std_array)[:end_ind_com]/nb_inst**0.5*2,
                             linewidth=1, alpha=.1, color=colors[auction_type])
            plt.plot(np.array(list(tota_max_com_i1_k1[cr][auction_type].keys())[:end_ind_com]),
                     np.array(list(tota_max_com_i1_k1[cr][auction_type].values()))[:end_ind_com]/nb_inst+np.array(std_array)[:end_ind_com]/nb_inst**0.5*2,
                     linewidth=1, alpha=.2, color=colors[auction_type])
            plt.plot(np.array(list(tota_max_com_i1_k1[cr][auction_type].keys())[:end_ind_com]),
                     np.array(list(tota_max_com_i1_k1[cr][auction_type].values()))[:end_ind_com]/nb_inst-np.array(std_array)[:end_ind_com]/nb_inst**0.5*2,
                     linewidth=1, alpha=.2, color=colors[auction_type])
            plt.plot(np.array(list(tota_max_com_i1_k1[cr][auction_type].keys())[:end_ind_com]),
                     np.array(list(tota_max_com_i1_k1[cr][auction_type].values()))[:end_ind_com]/nb_inst,
                     linewidth=1, alpha=1, linestyle="dashed", color=colors[auction_type])
        plt.legend()
        plt.grid(color = 'k', linestyle = '--', linewidth = 0.25)
        plt.legend(prop=font)
        plt.ylabel("Maximum Idleness (sec.)", fontsize="large", fontdict=font)
        plt.xlabel("Simulated Time (sec.)", fontsize="large", fontdict=font)
        plt.ylim(0.0, 3600.0)
        plt.xticks(fontsize=12, family='serif')
        plt.yticks(fontsize=12, family='serif')
        plt.tight_layout()
        plt.savefig(f"{root_folder}/{instance_dir}/max_idleness_cr_{cr}.pdf")
        plt.clf()
        print("Plot done.")

