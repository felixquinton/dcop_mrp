import pandas as pd
import json
import numpy as np
from matplotlib import pyplot as plt

root_folder = "PAAMS_auctions"
instance_dir = "grid_graphs_dim=7x7_obsprop=0.15_p60" #"caylus_graphs_large_events_from_1200_to_1500_p100"caylus_small_r5_p200 caylus_large_r10_p75grid_graphs_dim_7_events_from_1200_to_1500_p#grid_graphs_dim=7x7_obsprop=0.15_p60

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

auction_types = ["ours", "sheng", "no_com"]  # ]  # , "no_com", "sheng"

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
elif instance_dir == "star_graphs_nb_branch=10_branch_length=5_obsprop=0.15_p208":
        inst_to_avoid = []
        
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
elif "dim=7" in instance_dir :
    nb_waypoints = 49
        
print(f"Analyse de {instance_dir} avec {auction_types}")

cpt = 0

tota_data = {}
for auction_type in auction_types:
        print(f"Starting instances with com {auction_type} term")
        first_folder = f"{root_folder}/{instance_dir}/{auction_type}"
        end_ind_com = 3600
        tota_data[auction_type] = {i: 0 for i in range(end_ind_com)}
        for i in range(nb_inst):
            if i in inst_to_avoid: continue
            cpt += 1
            tmp_ind = str(i)
            folder_address = first_folder + "/" + tmp_ind
            folder_address = first_folder + "/" + tmp_ind
            file_address = folder_address + "/com_graph.csv"
            data = pd.read_csv(file_address, header=None)
            data_dic = {}
            data_dic[0.0] = [10] + [0]*9
            data_mean = {}
            data_max = {}
            previous_time = 0.0
            for ind, line in enumerate(data.iterrows()):
                    if round(line[1][0]) < 3600:
                        new_time = round(line[1][0])
                        read_json = json.loads(line[1][1])
                        data_dic[new_time] = [0]*10
                        for ind, cc in enumerate(read_json.values()):
                            data_dic[new_time][ind] = cc["size"]
                        previous_time = new_time
            xs = np.array(list(data_dic.keys()))
            ws = np.roll(xs, -1) - xs
            ws[-1] = 3600 - xs[-1]
            fig = plt.figure()
            ax = fig.add_axes([0,0,1,1])
            prev_hs = np.repeat(10, xs.shape)  # np.array(list(data_dic.values()))
            for i in range(10):
                hs = [h[i] for h in data_dic.values()]
                ax.bar(xs, hs, ws, bottom=prev_hs, align="edge")
                prev_hs += np.array(hs)
            plt.savefig(folder_address + "/histo_connected_components.pdf")
            plt.clf()
