import csv
import pandas as pd
import json
import numpy as np
import networkx as nx
from matplotlib import pyplot as plt

a=1

first_folder = "com"


speed = 1

nb_inst_com = 9
total_mean_com_i1_k1 = {}
total_min_com_i1_k1 = {}
total_cluster_size_com_i1_k1 = {}


for cr in [40, 50, 60, 75]:
    for pmul in ["0", "1", "2"]:
        total_mean_com_i1_k1[cr] = {}
        total_min_com_i1_k1[cr] = {}
        total_cluster_size_com_i1_k1[cr] = {}
        for auction_type in ["our", "sheng", "none"]:
            print(f"Starting instances with com {auction_type} term, pmul = {pmul} and cr = {cr}")
            first_folder = f"ICRA_tests_res/simu_icra_obs_{pmul}_cr_{cr}/{auction_type}"
            range_value = cr
            end_ind_com = 1200
            for i in range(1, nb_inst_com+1):
                tmp_ind = str(i)
                folder_address = first_folder + "/" + tmp_ind
                file_address = folder_address + "/com_graph_inst_5r_20w_r700_01.csv"
                data = pd.read_csv(file_address, header=None)
                first_line = data.iloc[[0]]
                first_time = round(first_line[0][0])
                list_seen_times = []
                data_dic = {i: 5 for i in range(end_ind_com)}
	        min_dic = {i: 1 for i in range(end_ind_com)}
	        size_dic = {i: {j: 0 for j in range(end_ind_com)} for i in range(1, 6)}

	        for ind, line in enumerate(data.iterrows()):
		        new_time = round(line[1][0])
		        delta_time = new_time - first_time
		        if delta_time not in list_seen_times:
			        list_seen_times.append(delta_time)
			        json_to_convert = json.loads(line[1][1])
			        json_to_convert["directed"] = False
			        first_graph = nx.readwrite.json_graph.node_link_graph(json_to_convert)
			        for j in range(1, 6):
				        first_graph.remove_node(j)
			        data_dic[delta_time] = min(data_dic[delta_time],  nx.algorithms.components.number_connected_components(first_graph))
			        min_dic[delta_time] = min([len(c) for c in nx.algorithms.components.connected_components(first_graph)])
			        for c in nx.algorithms.components.connected_components(first_graph):
				        size_dic[len(c)][delta_time] += 1
	        if delta_time < end_ind_com:
		        end_ind_com = delta_time
	        for t in data_dic:
                        total_mean_com_i1_k1[cr][t] += data_dic[t]
	        for t in min_dic:
		        total_min_com_i1_k1[cr][t] += min_dic[t]
	        for s in size_dic:
		        for t in size_dic[s]:
			        total_cluster_size_com_i1_k1[cr][s][t] += size_dic[s][t]
	        plt.plot(list(data_dic.keys()), list(data_dic.values()), label='Range=75, com, nb CC')
	        plt.legend()
	        plt.savefig(folder_address + "/nb_connected_components.png")
	        plt.clf()
	        plt.plot(list(min_dic.keys()), list(min_dic.values()), label='Range=75, com, min r in cc')
	        plt.legend()
	        plt.savefig(folder_address + "/smallest_cc.png")
	        plt.clf()
	        for s in size_dic:
		        plt.plot(list(size_dic[s].keys()), list(size_dic[s].values()), label='Range=75, com, nb of clusters size '+str(s))
		        plt.legend()
		        plt.savefig(folder_address + "/nb_cluster_of_size"+str(s)+".png")
		        plt.clf()
	        print("Plotted instance ", i)

        end_ind_com = int(end_ind_com)

        plt.plot(list(total_mean_com_i1_k1[cr].keys())[:end_ind_com], np.array(list(total_mean_com_i1_k1[cr].values()))[:end_ind_com]/nb_inst_com, label='Range='+str(cr)+', com, nb CC')
        plt.legend()
        plt.savefig(first_folder + "/mean_nb_connected_components.png")
        plt.clf()
        plt.plot(list(total_min_com_i1_k1[cr].keys())[:end_ind_com], np.array(list(total_min_com_i1_k1[cr].values()))[:end_ind_com]/nb_inst_com, label='Range='+str(cr)+', com, nb CC')
        plt.legend()
        plt.savefig(first_folder + "/mean_smallest_cc.png")
        plt.clf()
        """
        for s in total_cluster_size_com_i1_k1:
	        plt.plot(list(total_cluster_size_com_i1_k1[cr][s].keys()), list(total_cluster_size_com_i1_k1[cr][s].values()), label='Range='+str(cr)+', com, nb of clusters size '+str(s))
	        plt.legend()
	        plt.savefig(first_folder + "/nb_cluster_of_size"+str(s)+".png")
	        plt.clf()
	"""
        print("Moyenne com:", np.mean(np.array(list(total_mean_com_i1_k1[cr].values()))[:end_ind_com]/nb_inst_com))


total_mean_no_com_i1_k1 = {}
total_min_no_com_i1_k1 = {}
total_cluster_size_no_com_i1_k1 = {}
nb_inst_no_com = 9
for cr in [100, 75]:
        first_folder = "MRS_simu/simu_mrs_obs_05_cr_"+str(cr)+"/no_com"
        range_value = cr
        end_ind_no_com = 600
        total_mean_no_com_i1_k1[cr] = {i: 0 for i in range(end_ind_no_com)}
        total_min_no_com_i1_k1[cr] = {i: 0 for i in range(end_ind_no_com)}
        total_cluster_size_no_com_i1_k1[cr] = {i: {j: 0 for j in range(end_ind_no_com)} for i in range(1, 6)}

        for i in range(1, nb_inst_no_com+1):
	        tmp_ind = str(i)
	        folder_address = first_folder + "/" + tmp_ind
	        file_address = folder_address + "/com_graph_inst_5r_20w_r700_01.csv"
	        data = pd.read_csv(file_address, header=None)
	        first_line = data.iloc[[0]]
	        first_time = round(first_line[0][0])

	        list_seen_times = []

	        data_dic = {i: 5 for i in range(600)}
	        min_dic = {i: 1 for i in range(600)}
	        size_dic = {i: {j: 0 for j in range(600)} for i in range(1, 6)}

	        for ind, line in enumerate(data.iterrows()):
		        new_time = round(line[1][0])
		        delta_time = new_time - first_time
		        if delta_time not in list_seen_times:
			        list_seen_times.append(delta_time)
			        json_to_convert = json.loads(line[1][1])
			        json_to_convert["directed"] = False
			        first_graph = nx.readwrite.json_graph.node_link_graph(json_to_convert)
			        for j in range(1, 6):
				        first_graph.remove_node(j)
			        data_dic[delta_time] = min(data_dic[delta_time],  nx.algorithms.components.number_connected_components(first_graph))
			        min_dic[delta_time] = min([len(c) for c in nx.algorithms.components.connected_components(first_graph)])
			        for c in nx.algorithms.components.connected_components(first_graph):
				        size_dic[len(c)][delta_time] += 1
	        if delta_time < end_ind_no_com:
		        end_ind_no_com = delta_time
	        for t in data_dic:
                        total_mean_no_com_i1_k1[cr][t] += data_dic[t]
	        for t in min_dic:
		        total_min_no_com_i1_k1[cr][t] += min_dic[t]
	        for s in size_dic:
		        for t in size_dic[s]:
			        total_cluster_size_no_com_i1_k1[cr][s][t] += size_dic[s][t]
	        plt.plot(list(data_dic.keys()), list(data_dic.values()), label='Range=75, com, nb CC')
	        plt.legend()
	        plt.savefig(folder_address + "/nb_connected_no_components.png")
	        plt.clf()
	        plt.plot(list(min_dic.keys()), list(min_dic.values()), label='Range=75, com, min r in cc')
	        plt.legend()
	        plt.savefig(folder_address + "/smallest_cc.png")
	        plt.clf()
	        for s in size_dic:
		        plt.plot(list(size_dic[s].keys()), list(size_dic[s].values()), label='Range=75, com, nb of clusters size '+str(s))
		        plt.legend()
		        plt.savefig(folder_address + "/nb_cluster_of_size"+str(s)+".png")
		        plt.clf()
	        print("Plotted instance ", i)

        end_ind_no_com = int(end_ind_no_com)

        plt.plot(list(total_mean_no_com_i1_k1[cr].keys())[:end_ind_no_com], np.array(list(total_mean_no_com_i1_k1[cr].values()))[:end_ind_no_com]/nb_inst_no_com, label='Range='+str(cr)+', com, nb CC')
        plt.legend()
        plt.savefig(first_folder + "/mean_nb_connected_no_components.png")
        plt.clf()
        plt.plot(list(total_min_no_com_i1_k1[cr].keys())[:end_ind_no_com], np.array(list(total_min_no_com_i1_k1[cr].values()))[:end_ind_no_com]/nb_inst_no_com, label='Range='+str(cr)+', com, nb CC')
        plt.legend()
        plt.savefig(first_folder + "/mean_smallest_cc.png")
        plt.clf()
        """
        for s in total_cluster_size_no_com_i1_k1:
	        plt.plot(list(total_cluster_size_no_com_i1_k1[cr][s].keys()), list(total_cluster_size_no_com_i1_k1[cr][s].values()), label='Range='+str(cr)+', com, nb of clusters size '+str(s))
	        plt.legend()
	        plt.savefig(first_folder + "/nb_cluster_of_size"+str(s)+".png")
	        plt.clf()
	"""
        print("Moyenne com:", np.mean(np.array(list(total_mean_no_com_i1_k1[cr].values()))[:end_ind_no_com]/nb_inst_no_com))
##############
# Final plot #
##############

#Mean

# C=1, a=1
cpt = 0
cl = ["b", "orange", "g", "r", "c", "m", "y", "k"]
for cr in total_mean_com_i1_k1:
        plt.plot(np.array(list(total_mean_com_i1_k1[cr].keys())[:end_ind_com]), np.array(list(total_mean_com_i1_k1[cr].values()))[:end_ind_com]/nb_inst_com, linewidth=1, color=cl[cpt], alpha=.5)

        # Regression polynomiale
        mymodel = np.poly1d(np.polyfit(np.array(list(total_mean_com_i1_k1[cr].keys())[:end_ind_com]),
                    np.array(list(total_mean_com_i1_k1[cr].values()))[:end_ind_com]/nb_inst_com, 3))
        myline = np.linspace(1, end_ind_com, 1000)
        plt.plot(myline, mymodel(myline), color=cl[cpt], label='Sheng, Com range = '+str(cr))
        cpt += 1
        plt.plot(np.array(list(total_mean_no_com_i1_k1[cr].keys())[:end_ind_no_com]), np.array(list(total_mean_no_com_i1_k1[cr].values()))[:end_ind_no_com]/nb_inst_no_com, linewidth=1, color=cl[cpt], alpha=.5)

        # Regression polynomiale
        mymodel = np.poly1d(np.polyfit(np.array(list(total_mean_no_com_i1_k1[cr].keys())[:end_ind_no_com]),
                    np.array(list(total_mean_no_com_i1_k1[cr].values()))[:end_ind_no_com]/nb_inst_no_com, 3))
        myline = np.linspace(1, end_ind_no_com, 1000)
        plt.plot(myline, mymodel(myline), color=cl[cpt], label='No com, Com range = '+str(cr))
        cpt += 1

plt.xlabel("Time (sec)")
plt.ylabel("Number of Connected Components")
plt.legend()
plt.savefig("MRS_plot/mean_nbcc_obstacle_obstruction_05.pdf")
plt.clf()
"""
#Min

# C=1, a=1
plt.plot(np.array(list(total_min_com_i1_k1.keys())[:end_ind_com]), np.array(list(total_min_com_i1_k1.values()))[:end_ind_com]/nb_inst_com, linewidth=1, alpha=.5, color='cyan')

# Regression polynomiale
mymodel = np.poly1d(np.polyfit(np.array(list(total_min_com_i1_k1.keys())[:end_ind_com]),
                    np.array(list(total_min_com_i1_k1.values()))[:end_ind_com]/nb_inst_com, 4))
myline = np.linspace(1, end_ind_com, 1000)
plt.plot(myline, mymodel(myline), label='C=1, a='+str(a)+', r='+str(range_value)+', deg=4', color='cyan')

# C=1, a=0
plt.plot(np.array(list(total_min_no_com.keys())[:end_ind_com]), np.array(list(total_min_no_com.values()))[:end_ind_com]/nb_inst, linewidth=1, alpha=.5, color='red')

# Regression polynomiale
mymodel = np.poly1d(np.polyfit(np.array(list(total_min_no_com.keys())[:end_ind_com]),
                    np.array(list(total_min_no_com.values()))[:end_ind_com]/nb_inst, 4))
myline = np.linspace(1, end_ind_com, 1000)
plt.plot(myline, mymodel(myline), label='C=1, a='+str(a)+', r='+str(range_value)+', deg=4', color='red')

plt.xlabel("Time (sec)")
plt.ylabel("# Connected Components")
plt.legend()
plt.savefig("min nb robot in cc, r=75, nb=16, rep=2.png")
plt.clf()

# Number of clusters of size n

for s in total_cluster_size_com_i1_k1:
	# C=1, a=1
	plt.plot(np.array(list(total_cluster_size_com_i1_k1[s].keys())), np.array(list(total_cluster_size_com_i1_k1[s].values()))/nb_inst_com, linewidth=1, alpha=.5, color='cyan')

	# Regression polynomiale
	mymodel = np.poly1d(np.polyfit(np.array(list(total_cluster_size_com_i1_k1[s].keys())[:end_ind_com]),
		            np.array(list(total_cluster_size_com_i1_k1[s].values()))[:end_ind_com]/nb_inst_com, 2))
	myline = np.linspace(1, end_ind_com, 1000)
	plt.plot(myline, mymodel(myline), label='a='+str(a)+', r='+str(range_value)+', deg=2', color='cyan')

	# C=1, a=0
	plt.plot(np.array(list(total_cluster_size_no_com[s].keys())), np.array(list(total_cluster_size_no_com[s].values()))/nb_inst, linewidth=1, alpha=.5, color='red')

	# Regression polynomiale
	mymodel = np.poly1d(np.polyfit(np.array(list(total_cluster_size_no_com[s].keys())[:end_ind_com]),
		            np.array(list(total_cluster_size_no_com[s].values()))[:end_ind_com]/nb_inst, 2))
	myline = np.linspace(1, end_ind_com, 1000)
	plt.plot(myline, mymodel(myline), label='a=0, r='+str(range_value)+', deg=2', color='red')
	plt.legend()
	plt.savefig(first_folder + "/" + second_folder + "/nb_cluster_of_size"+str(s)+".png")

	plt.xlabel("Time (sec)")
	plt.ylabel("# Clusters of Size " + str(s))
	plt.legend()
	plt.savefig("nb of clusters size, r=75, nb=16, rep=2.png")
	plt.clf()
"""

print("NB CC plots OK.")
