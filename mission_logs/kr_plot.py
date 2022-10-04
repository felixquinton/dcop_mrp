import csv
import pandas as pd
import json
import numpy as np
from matplotlib import pyplot as plt

root_folder = "PAAMS_auctions"
instance_dir = "caylus_graphs_large_events_from_1200_to_1500_p" #"caylus_graphs_large_events_from_1200_to_1500_p100"caylus_small_r5_p200 caylus_large_r10_p75grid_graphs_dim_7_events_from_1200_to_1500_p

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

auction_types = ["ours"]  # ]  # 

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
        inst_to_avoid = [31, 33]
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

for cr in [100, 200]:
        tota_data_com[cr] = {}
        tota_data_max_com[cr] = {}
        tota_mean_com_i1_k1[cr] = {}
        tota_max_com_i1_k1[cr] = {}
        end_ind_list = []
        for auction_type in auction_types:
                print(f"Starting instances with com {auction_type} term")
                first_folder = f"{root_folder}/{instance_dir}{cr}/{auction_type}"
                end_ind_com = 3600
                tot_nb = {}
                for i in range(nb_inst):
                    inst_nb = {}
                    for r in range(1, 11):
                        if i in inst_to_avoid: continue
                        folder_address = f"{first_folder}/{i}"
                        file_address = folder_address + f"/com_cost_robot_{r}.csv"
                        try:
                            data = pd.read_csv(file_address, header=None)
                        except:
                            pass
                        values = data[3]
                        for ind, v in enumerate(values):
                            if v <= -1:
                                values[ind] += 1
                        count = values.round(3).value_counts().sort_index()
                        for v, n in count.items():
                            nn = tot_nb.get(v, 0) + n
                            tot_nb[v] = nn
                            nn = inst_nb.get(v, 0) + n
                            inst_nb[v] = nn                
                        """
                        fig, ax = plt.subplots(figsize = (10,4))
                        ax.bar(np.arange(len(count)), count.values)
                        ax.set_xticks(np.arange(len(count)))
                        ax.set_xticklabels([round(v, 3) for v in count.keys()])
                        plt.xticks(rotation=45)
                        plt.savefig(folder_address + f"/kr_occurences_{r}.pdf")
                        plt.clf()
                        """
                    series = pd.Series(inst_nb).sort_index()
                    fig, ax = plt.subplots(figsize = (10,4))
                    ax.bar(np.arange(len(series)), series.values)
                    ax.set_xticks(np.arange(len(series)))
                    ax.set_xticklabels([round(v, 3) for v in series.keys()])
                    plt.xticks(rotation=45)
                    plt.savefig(folder_address + f"/kr_occurences_inst_{i}.pdf")
                    plt.clf()
                print(tot_nb)
                series = pd.Series(tot_nb).sort_index()
                print(series)
                fig, ax = plt.subplots(figsize = (10,4))
                ax.bar(np.arange(len(series)), series.values)
                ax.set_xticks(np.arange(len(series)))
                ax.set_xticklabels([round(v, 3) for v in series.keys()])
                plt.xticks(rotation=45)
                plt.savefig(first_folder + "/kr_occurences.pdf")
                plt.clf()
                
raise TypeError

