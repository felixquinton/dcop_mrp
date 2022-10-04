import csv
import pandas as pd
import json
import numpy as np
from matplotlib import pyplot as plt

root_folder = "PAAMS_auctions"
instance_dir = "caylus_graphs_large_events_from_1200_to_1500_p200" #"grid_10x10_r10"caylus_small_r5_p200 caylus_large_r10_p75

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
if instance_dir == "caylus_large_r10":
        inst_to_avoid = [9, 29, 0] + [6, 19, 23, 33, 42]# [4, 18, 19, 30, 44, 5, 29, 20, 33, 34, 9]# [5, 24, 29, 30, 33, 38, 25, 41, 44]
elif instance_dir == "star_5x5_r5":
        inst_to_avoid = [42, 44, 60] + [8, 12, 17, 19, 39, 40, 43, 48, 50, 57, 61, 63] # 2, 9, 12
elif instance_dir == "star_10x5_r10":
        inst_to_avoid = [19, 31, 33, 52, 57] + [i for i in range(10)]
elif instance_dir == "grid_10x10_r10":
        inst_to_avoid = [1, 2, 3, 5, 6, 7, 8, 9, 4, 15, 29, 32, 49, 55, 63] #[0, 2, 10, 16, 25, 27, 29, 31, 39]# sheng[8, 9] # no com[0, 2, 10, 16, 25] # ours :[0, 34, 38, 48, 40]
elif instance_dir == "grid_5x5_r5":
        inst_to_avoid = []
elif instance_dir == "caylus_small_r5":
        inst_to_avoid = [0, 3, 4, 7, 10, 11, 18, 21, 32, 33, 34, 35, 41, 42, 54, 56] + [26, 51, 64, 5, 30, 47, 49, 52, 58, 59, 44, 45, 50, 29] #2, 9, 12
elif instance_dir == "caylus_small_r5_p200":
        inst_to_avoid = [8, 10, 12, 13, 14, 16, 52, 53, 62, 25, 4, 17, 19, 21, 23, 27, 29, 31, 33, 40, 42, 44, 46, 48, 6, 50, 55, 58, 64]  # [12, 18, 19, 32]  # [5, 21, 12, 19] + [] #2, 9, 12
elif instance_dir == "caylus_small_r5_p100":
        inst_to_avoid = [10, 15, 28, 31, 44, 48, 62, 4, 24, 27, 37, 48, 49]  # [1, 9, 11, 14, 45]  # + ours # [1, 9, 11, 14] # - ours + [] #2, 9, 12
elif instance_dir == "caylus_small_r5_p75":
        inst_to_avoid = [4, 7, 21, 27, 34, 39, 43, 50, 57]  # [1, 5, 12, 18, 25]  # + ours # [1, 9, 11, 14] # - ours + [] #2, 9, 12
elif instance_dir == "caylus_large_r10_p75":
        inst_to_avoid = [1, 3, 6, 7, 9, 13, 17, 18, 20, 24, 29, 31, 36, 37, 43, 44, 45]
elif instance_dir == "caylus_large_r10_p100":
        inst_to_avoid = [1, 3, 6, 17, 18, 20, 29, 31, 36, 37, 44, 45, 48, 4, 27, 21, 47, 38]
elif instance_dir == "caylus_large_r10_p200":
        inst_to_avoid = [4, 18, 19, 20, 21, 23, 27, 35, 36, 17, 44, 47, 0, 16, 45, 7, 26, 32, 3, 15]
elif instance_dir == "caylus_graphs_large_events_from_600_to_900":
        inst_to_avoid = [5]
elif instance_dir == "caylus_graphs_large_events_from_900_to_1200":
        inst_to_avoid = [2, 3, 6]
elif instance_dir == "caylus_graphs_large_events_from_1200_to_1500_p60":
        inst_to_avoid = [4]
elif instance_dir == "caylus_graphs_large_events_from_1200_to_1500_p75":
        inst_to_avoid = []
elif instance_dir == "caylus_graphs_large_events_from_1200_to_1500_p100":
        inst_to_avoid = []
        
nb_inst_dic = {"our+sheng": 15, "ours": nb_inst-len(inst_to_avoid), "sheng": nb_inst-len(inst_to_avoid), "no_com": nb_inst-len(inst_to_avoid)}
        
nb_waypoints = 0
if instance_dir == "star_10x5_r10":
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
        plt.ylabel("Oisiveté Maximale (sec.)", fontsize="large", fontdict=font)
        plt.xlabel("Temps Simulé (sec.)", fontsize="large", fontdict=font)
        plt.xticks(fontsize=12, family='serif')
        plt.yticks(fontsize=12, family='serif')
        plt.savefig(f"{root_folder}/{instance_dir}/max_idleness_cr_{cr}.pdf")
        plt.clf()
        print("Plot done.")

raise TypeError


tota_data_no_com = {}
tota_data_max_no_com = {}
tota_mean_no_com_i1_k1 = {}
tota_max_no_com_i1_k1 = {}
for cr in [100, 50, 70]:
        print("Starting instances with sheng com and cr = "+str(cr))
        first_folder = "ICRA_tests_res/simu_icra_obs_"+str(pmul)+"_cr_"+str(cr)+"/none"
        end_ind_no_com = 600
        tota_data_no_com[cr] = {i: [] for i in range(end_ind_no_com)}
        tota_data_max_no_com[cr] = {i: [] for i in range(end_ind_no_com)}
        tota_mean_no_com_i1_k1[cr] = {i: 0 for i in range(end_ind_no_com)}
        tota_max_no_com_i1_k1[cr] = {i: 0 for i in range(end_ind_no_com)}
        for i in range(1, nb_inst+1):
	        tmp_ind = str(i)
	        folder_address = first_folder + "/" + tmp_ind
	        folder_address = first_folder + "/" + tmp_ind
	        file_address = folder_address + "/idle_inst_5r_20w_r700_01.csv"
	        data = pd.read_csv(file_address, header=None)
	        first_line = data.iloc[[0]]
	        first_dic = json.loads(first_line[3][0])
	        first_time = round(first_line[1][0])
	        data_dic = {}
	        data_dic[0.0] = {str(w): 1 for w in range(1, 41)}

	        data_mean = {}
	        data_max = {}

	        previous_time = 0.0

	        for ind, line in enumerate(data.iterrows()):
	                if round(line[1][1]) < 600:
		                new_time = round(line[1][1])
		                read_json = json.loads(line[1][3])
		                if new_time-previous_time > 1:
			                adding_cpt = 1
			                for t in range(int(previous_time+1), int(new_time)):
				                data_dic[float(t)] = {w: t - data_dic[previous_time][w] + adding_cpt
					                       for w in data_dic[previous_time]}
				                data_mean[float(t)] = np.mean(list(data_dic[float(t)].values()))
				                data_max[float(t)] = np.max(list(data_dic[float(t)].values()))
				                adding_cpt += 1
		                data_dic[new_time] = {w: new_time - data_dic[new_time-1][w] + 1 for w in data_dic[previous_time]}
		                for w in read_json:
			                if not w == "mean" and not w == "max":
				                if read_json[w] < data_dic[new_time][w]:
					                data_dic[new_time][w] = new_time - read_json[w]

		                data_mean[new_time] = np.mean(list(data_dic[new_time].values()))
		                data_max[new_time] = np.max(list(data_dic[new_time].values()))
		                previous_time = new_time
	        if previous_time < end_ind_no_com:
		        end_ind_no_com = previous_time
	        print(i, previous_time)
	        for t in data_mean:
		        tota_mean_no_com_i1_k1[cr][t] += data_mean[t]
	        for t in data_max:
		        tota_max_no_com_i1_k1[cr][t] += data_max[t]
	        for t in data_mean:
		        tota_data_no_com[cr][t].append(data_mean[t])
	        for t in data_max:
		        tota_data_max_no_com[cr][t].append(data_max[t])
	        plt.plot(list(data_mean.keys()), list(data_mean.values()), linewidth=.25, label='Range=50, sheng com, mean')
	        plt.legend()
	        plt.savefig(folder_address + "/mean_idleness.pdf")
	        plt.clf()
	        plt.plot(list(data_max.keys()), list(data_max.values()), linewidth=.25, label='Range=50, sheng com, max')
	        plt.legend()
	        plt.savefig(folder_address + "/max_idleness.pdf")
	        plt.clf()


        end_ind_no_com = int(end_ind_no_com)
        print(end_ind_no_com)

        plt.plot(list(tota_mean_no_com_i1_k1[cr].keys())[:end_ind_no_com], np.array(list(tota_mean_no_com_i1_k1[cr].values()))[:end_ind_no_com]/nb_inst, label='Range=50, sheng com, mean')

        # Regression polynomiale
        mymodel = np.poly1d(np.polyfit(list(tota_mean_no_com_i1_k1[cr].keys())[:end_ind_no_com],
                            np.array(list(tota_mean_no_com_i1_k1[cr].values()))[:end_ind_no_com]/nb_inst, 3))
        myline = np.linspace(1, end_ind_no_com, 1000)
        plt.plot(myline, mymodel(myline))

        plt.legend()
        plt.savefig(first_folder + "/mean_idleness.pdf")
        plt.clf()

        plt.plot(list(tota_max_no_com_i1_k1[cr].keys())[:end_ind_no_com], np.array(list(tota_max_no_com_i1_k1[cr].values()))[:end_ind_no_com]/nb_inst, label='Range=50, sheng com, max')

        # Regression polynomiale
        mymodel = np.poly1d(np.polyfit(list(tota_max_no_com_i1_k1[cr].keys())[:end_ind_no_com],
                            np.array(list(tota_max_no_com_i1_k1[cr].values()))[:end_ind_no_com]/nb_inst, 3))
        myline = np.linspace(1, end_ind_no_com, 1000)
        plt.plot(myline, mymodel(myline))

        plt.legend()
        plt.savefig(first_folder + "/max_idleness.pdf")
        plt.clf()
##############
# Final plot #
##############

cpt = 0
colors = ['b', 'c', 'g', 'y', 'r', 'm']

#Mean

for cr in tota_mean_com_i1_k1:
        plt.plot(np.array(list(tota_mean_com_i1_k1[cr].keys())[:end_ind_com]), np.array(list(tota_mean_com_i1_k1[cr].values()))[:end_ind_com]/nb_inst, linewidth=1, alpha=1, label='Com range = '+str(cr)+', Our term', color=colors[cpt])


        # sigma
        std_array = []
        for v in tota_data_com[cr].values():
                std_array.append(np.std(list(v)))
        print(np.array(list(tota_mean_com_i1_k1[cr].values()))[:end_ind_com], nb_inst)
        plt.fill_between(np.array(list(tota_mean_com_i1_k1[cr].keys())[:end_ind_com]),
                 np.array(list(tota_mean_com_i1_k1[cr].values()))[:end_ind_com]/nb_inst+np.array(std_array)[:end_ind_com],
                         np.array(list(tota_mean_com_i1_k1[cr].values()))[:end_ind_com]/nb_inst-np.array(std_array)[:end_ind_com], linewidth=1, alpha=.1, color=colors[cpt])
        plt.plot(np.array(list(tota_mean_com_i1_k1[cr].keys())[:end_ind_com]),
                 np.array(list(tota_mean_com_i1_k1[cr].values()))[:end_ind_com]/nb_inst+np.array(std_array)[:end_ind_com], linewidth=1, alpha=.2, color=colors[cpt])
        plt.plot(np.array(list(tota_mean_com_i1_k1[cr].keys())[:end_ind_com]),
                 np.array(list(tota_mean_com_i1_k1[cr].values()))[:end_ind_com]/nb_inst-np.array(std_array)[:end_ind_com], linewidth=1, alpha=.2, color=colors[cpt])

        cpt += 1
        print(cr, end_ind_no_com)
        plt.plot(np.array(list(tota_mean_no_com_i1_k1[cr].keys())[:end_ind_no_com]), np.array(list(tota_mean_no_com_i1_k1[cr].values()))[:end_ind_no_com]/nb_inst, linewidth=1, alpha=1, label='Com range = '+str(cr)+', sheng com term', linestyle="dashed", color=colors[cpt])

        # sigma
        std_array = []
        for v in tota_data_no_com[cr].values():
                std_array.append(np.std(list(v)))

        plt.fill_between(np.array(list(tota_mean_no_com_i1_k1[cr].keys())[:end_ind_no_com]),
                 np.array(list(tota_mean_no_com_i1_k1[cr].values()))[:end_ind_no_com]/nb_inst+np.array(std_array)[:end_ind_no_com],
                         np.array(list(tota_mean_no_com_i1_k1[cr].values()))[:end_ind_no_com]/nb_inst-np.array(std_array)[:end_ind_no_com], linewidth=1, alpha=.1, color=colors[cpt])
        plt.plot(np.array(list(tota_mean_no_com_i1_k1[cr].keys())[:end_ind_no_com]),
                 np.array(list(tota_mean_no_com_i1_k1[cr].values()))[:end_ind_no_com]/nb_inst+np.array(std_array)[:end_ind_no_com], linewidth=1, alpha=.2, color=colors[cpt])
        plt.plot(np.array(list(tota_mean_no_com_i1_k1[cr].keys())[:end_ind_no_com]),
                 np.array(list(tota_mean_no_com_i1_k1[cr].values()))[:end_ind_no_com]/nb_inst-np.array(std_array)[:end_ind_no_com], linewidth=1, alpha=.2, color=colors[cpt])

        cpt += 1

plt.ylim((0,1800))
plt.xlabel("Temps (sec)")
plt.ylabel("Oisiveté moyenne (sec)")
plt.legend()
plt.savefig("ICRA_tests_res/mean_idleness_pmul_"+pmul+"_our_com.pdf")
plt.clf()
"""
# Regression polynomiale
mymodel = np.poly1d(np.polyfit(np.array(list(tota_mean_com_i1_k1.keys())[:end_ind_com]),
                    np.array(list(tota_mean_com_i1_k1.values()))[:end_ind_com]/nb_inst, 3))
myline = np.linspace(1, end_ind_com, 1000)

plt.plot(myline, mymodel(myline), label='C=1, a=1, r=50, deg=3', color='blue')


# sigma
std_array = []
for v in tota_data_com[cr].values():
        std_array.append(np.std(list(v)))
        plt.fill_between(np.array(list(tota_mean_com_i1_k1[cr].keys())[:end_ind_com]),
         np.array(list(tota_mean_com_i1_k1[cr].values()))[:end_ind_com]/nb_inst+np.array(std_array)[:end_ind_com],
                 np.array(list(tota_mean_com_i1_k1[cr].values()))[:end_ind_com]/nb_inst-np.array(std_array)[:end_ind_com], linewidth=1, alpha=.1, color='blue')
        plt.plot(np.array(list(tota_mean_com_i1_k1[cr].keys())[:end_ind_com]),
                 np.array(list(tota_mean_com_i1_k1[cr].values()))[:end_ind_com]/nb_inst+np.array(std_array)[:end_ind_com], linewidth=1, alpha=.2, color='blue', linestyle='--', label="Avec terme com., +/- sigma")
        plt.plot(np.array(list(tota_mean_com_i1_k1[cr].keys())[:end_ind_com]),
                 np.array(list(tota_mean_com_i1_k1[cr].values()))[:end_ind_com]/nb_inst-np.array(std_array)[:end_ind_com], linewidth=1, alpha=.2, color='blue', linestyle='--')
"""


# Max

cpt = 0

for cr in tota_mean_com_i1_k1:

        plt.plot(np.array(list(tota_max_com_i1_k1[cr].keys())[:end_ind_com]), np.array(list(tota_max_com_i1_k1[cr].values()))[:end_ind_com]/nb_inst, linewidth=1, alpha=1, label='Com range = '+str(cr)+', our term', color=colors[cpt])


        # sigma
        std_array = []
        for v in tota_data_com[cr].values():
                std_array.append(np.std(list(v)))

        plt.fill_between(np.array(list(tota_max_com_i1_k1[cr].keys())[:end_ind_com]),
                 np.array(list(tota_max_com_i1_k1[cr].values()))[:end_ind_com]/nb_inst+np.array(std_array)[:end_ind_com],
                         np.array(list(tota_max_com_i1_k1[cr].values()))[:end_ind_com]/nb_inst-np.array(std_array)[:end_ind_com], linewidth=1, alpha=.1, color=colors[cpt])
        plt.plot(np.array(list(tota_max_com_i1_k1[cr].keys())[:end_ind_com]),
                 np.array(list(tota_max_com_i1_k1[cr].values()))[:end_ind_com]/nb_inst+np.array(std_array)[:end_ind_com], linewidth=1, alpha=.2, color=colors[cpt])
        plt.plot(np.array(list(tota_max_com_i1_k1[cr].keys())[:end_ind_com]),
                 np.array(list(tota_max_com_i1_k1[cr].values()))[:end_ind_com]/nb_inst-np.array(std_array)[:end_ind_com], linewidth=1, alpha=.2, color=colors[cpt])

        cpt += 1
        plt.plot(np.array(list(tota_max_no_com_i1_k1[cr].keys())[:end_ind_no_com]), np.array(list(tota_max_no_com_i1_k1[cr].values()))[:end_ind_no_com]/nb_inst, linewidth=1, alpha=1, label='Com range = '+str(cr)+', sheng com term', linestyle="dashed", color=colors[cpt])


        # sigma
        std_array = []
        for v in tota_data_no_com[cr].values():
                std_array.append(np.std(list(v)))
        plt.fill_between(np.array(list(tota_max_no_com_i1_k1[cr].keys())[:end_ind_no_com]),
                 np.array(list(tota_max_no_com_i1_k1[cr].values()))[:end_ind_no_com]/nb_inst+np.array(std_array)[:end_ind_no_com],
                         np.array(list(tota_max_no_com_i1_k1[cr].values()))[:end_ind_no_com]/nb_inst-np.array(std_array)[:end_ind_no_com], linewidth=1, alpha=.1, color=colors[cpt])
        plt.plot(np.array(list(tota_max_no_com_i1_k1[cr].keys())[:end_ind_no_com]),
                 np.array(list(tota_max_no_com_i1_k1[cr].values()))[:end_ind_no_com]/nb_inst+np.array(std_array)[:end_ind_no_com], linewidth=1, alpha=.2, color=colors[cpt])
        plt.plot(np.array(list(tota_max_no_com_i1_k1[cr].keys())[:end_ind_no_com]),
                 np.array(list(tota_max_no_com_i1_k1[cr].values()))[:end_ind_no_com]/nb_inst-np.array(std_array)[:end_ind_no_com], linewidth=1, alpha=.2, color=colors[cpt])

        cpt += 1



plt.ylim((0,1800))
plt.xlabel("Temps (sec)")
plt.ylabel("Oisiveté maximale (sec)")
plt.legend()
plt.savefig("ICRA_tests_res/max_idleness_pmul_"+pmul+"_our_com.pdf")
plt.clf()
print("Idleness plots OK.")

for i in tota_mean_no_com[:end_ind_no_com]:
	print(i)
print()
for i in tota_mean_com_i1_k1[:end_ind_com]:
	print(i)
print()
for i in tota_max_no_com[:end_ind_no_com]:
	print(i)
print()
for i in tota_max_com_i1_k1[:end_ind_com]:
	print(i)
print()
