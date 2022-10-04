import csv
import pandas as pd
import json
import numpy as np
from matplotlib import pyplot as plt

first_folder = "ICRA_tests"

pmul="none"

nb_inst = 50

speed = 1

tota_data_com = {}
tota_data_max_com = {}
tota_mean_com_i1_k1 = {}
tota_max_com_i1_k1 = {}

colors = {"our": 'b', "sheng": 'r', "none": "g", "oursheng": "m"}
colors_per_pmul = {
    "our": {"2": "b", "1": "cyan", "0": "mediumpurple"},
    "sheng": {"2": "r", "1": "orangered", "0": "lightcoral"},
    "oursheng": {"2": "m", "1": "pink", "0": "mediumorchid"},
    "none": {"2": "g", "1": "lime", "0": "darkgreen"}}
reg_colors = {"our": "c", "sheng": 'orangered', "none": "lime",
              "oursheng": "pink"}

nb_inst_dic = {"oursheng": 50, "our": 50, "sheng": 50, "none": 50}

for pmul in ["2", "1", "0"]:
        for cr in [50, 60, 75]:
                tota_data_com[pmul] = {}
                tota_data_max_com[pmul] = {}
                tota_mean_com_i1_k1[pmul] = {}
                tota_max_com_i1_k1[pmul] = {}
                for auction_type in ["our", "sheng", "oursheng", "none"]:
                        print(f"Starting instances with com {auction_type} term and pmul = {pmul}")
                        first_folder = f"ICRA_tests_res/{cr}_cr_{pmul}_pmul/{auction_type}"
                        end_ind_com = 1200
                        tota_data_com[pmul][auction_type] = {i: [] for i in range(end_ind_com)}
                        tota_data_max_com[pmul][auction_type] = {i: [] for i in range(end_ind_com)}
                        tota_mean_com_i1_k1[pmul][auction_type] = {i: 0 for i in range(end_ind_com)}
                        tota_max_com_i1_k1[pmul][auction_type] = {i: 0 for i in range(end_ind_com)}
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
                            data_dic[0.0] = {str(w): 1 for w in range(1, 22)}
                            data_mean = {}
                            data_max = {}
                            previous_time = 0.0
                            for ind, line in enumerate(data.iterrows()):
                                    if round(line[1][1]) < 1200:
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
                                        data_dic[new_time] = {w: data_dic[new_time-1][w] + 1 for w in data_dic[previous_time]}
                                        for w in read_json:
                                            if not w == "mean" and not w == "max":
                                                if new_time - read_json[w] < data_dic[new_time][w]:
                                                    data_dic[new_time][w] = new_time - read_json[w]
                                        data_mean[new_time] = np.mean(list(data_dic[new_time].values()))
                                        data_max[new_time] = np.max(list(data_dic[new_time].values()))
                                        previous_time = new_time
                            if previous_time < end_ind_com:
                                end_ind_com = previous_time
                            print(i, previous_time)
                            for t in data_mean:
                                tota_mean_com_i1_k1[pmul][auction_type][t] += data_mean[t]
                            for t in data_max:
                                tota_max_com_i1_k1[pmul][auction_type][t] += data_max[t]
                            for t in data_mean:
                                tota_data_com[pmul][auction_type][t].append(data_mean[t])
                            for t in data_max:
                                tota_data_max_com[pmul][auction_type][t].append(data_max[t])
                        end_ind_com = int(end_ind_com)
                        print(end_ind_com)
for auction_type in ["our", "sheng", "oursheng", "none"]:
    for pmul in ["2", "1", "0"]:
        plt.plot(list(tota_mean_com_i1_k1[pmul][auction_type].keys())[:end_ind_com],
                 np.array(list(tota_mean_com_i1_k1[pmul][auction_type].values()))[:end_ind_com]/nb_inst,
                 label=f"Obstacles={pmul}, com={auction_type}, mean", color=colors_per_pmul[auction_type][pmul])#colors[auction_type])

    plt.legend()
    plt.savefig(f"ICRA_tests_res/mean_idleness_pmul_{pmul}_{auction_type}_com.pdf")
    plt.clf()


for auction_type in ["our", "sheng", "oursheng", "none"]:
    for pmul in ["2", "1", "0"]:
        plt.plot(list(tota_max_com_i1_k1[pmul][auction_type].keys())[:end_ind_com],
                 np.array(list(tota_max_com_i1_k1[pmul][auction_type].values()))[:end_ind_com]/nb_inst,
                 label=f"Obstacles={pmul}, com={auction_type}, max", color=colors_per_pmul[auction_type][pmul])#colors[auction_type])

    plt.legend()
    plt.savefig(f"ICRA_tests_res/mean_idleness_pmul_{pmul}_{auction_type}_com.pdf")
    plt.clf()

print("Plot done.")

raise TypeError


tota_data_no_com = {}
tota_data_max_no_com = {}
tota_mean_no_com_i1_k1 = {}
tota_max_no_com_i1_k1 = {}
for cr in [100, 50, 70]:
        print("Starting instances with sheng com and cr = "+str(cr))
        first_folder = "ICRA_tests/simu_icra_obs_"+str(pmul)+"_cr_"+str(cr)+"/none"
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
plt.savefig("ICRA_tests/mean_idleness_pmul_"+pmul+"_our_com.pdf")
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
plt.savefig("ICRA_tests/max_idleness_pmul_"+pmul+"_our_com.pdf")
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
