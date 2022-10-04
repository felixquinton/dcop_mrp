import csv
import pandas as pd
import json
import numpy as np
from matplotlib import pyplot as plt

first_folder = "ICRA_tests"

pmul="no_com"

nb_inst_global = 30

speed = 1

tota_data_com = {}
tota_data_max_com = {}
tota_mean_com_i1_k1 = {}
tota_max_com_i1_k1 = {}

colors = {"ours": 'b', "sheng": 'r', "no_com": "g", "ourssheng": "m"}
colors_per_pmul = {
    "ours": {"1": "b", "0": "cyan", "no_com": "mediumpurple"},
    "sheng": {"1": "r", "0": "orangered", "no_com": "lightcoral"},
    "no_com": {"1": "g", "0": "lime", "no_com": "darkgreen"}}
reg_colors = {"ours": "c", "sheng": 'orangered', "no_com": "lime",
              "ourssheng": "pink"}
auction_formula = {"ours": r'$I + K$',
                   "sheng": r'$I + S$',
                   "ourssheng": r'$I + K + S$',
                   "no_com": r'$I$', }

nb_inst_dic = {"ourssheng": 30, "ours": 24, "sheng": 25, "no_com": 26}

cr = 250 
tota_data_com[cr] = {}
tota_data_max_com[cr] = {}
tota_mean_com_i1_k1[cr] = {}
tota_max_com_i1_k1[cr] = {}
end_ind_list = []
for auction_type in ["ours", "sheng", "no_com"]:
        print(f"Starting instances")
        first_folder = f"IROS_tests/grid_5x5_r5/{auction_type}"
        end_ind_com = 1500
        max_distrib = {}
        tota_data_com[cr][auction_type] = {i: [] for i in range(end_ind_com)}
        tota_data_max_com[cr][auction_type] = {i: [] for i in range(end_ind_com)}
        tota_mean_com_i1_k1[cr][auction_type] = {i: 0 for i in range(end_ind_com)}
        tota_max_com_i1_k1[cr][auction_type] = {i: 0 for i in range(end_ind_com)}
        for i in range(nb_inst_global):
            if auction_type == "no_com" and i in [3, 10, 18, 22]:
                continue
            if auction_type == "sheng" and i in [3, 10, 18, 22, 28]:
                continue
            if auction_type == "ours" and i in [3, 10, 12, 18, 22, 28]:
                continue
            nb_inst = nb_inst_dic[auction_type]
            tmp_ind = str(i)
            folder_address = first_folder + "/" + tmp_ind
            folder_address = first_folder + "/" + tmp_ind
            file_address = folder_address + "/idle_inst_5r_20w_r700_01.csv"
            data = pd.read_csv(file_address, header=None)
            alloc_add = folder_address + "/allocation.csv"
            data_alloc = pd.read_csv(alloc_add, header=None)
            first_line = data.iloc[[0]]
            first_dic = json.loads(first_line[3][0])
            first_time = round(first_line[1][0])
            data_dic = {}
            data_dic[0.0] = {str(w): 1 for w in range(25)}
            data_mean = {}
            data_max = {}
            previous_time = 0.0
            for ind, line in enumerate(data.iterrows()):
                    if round(line[1][1]) < 1500:
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
            """
            for j in range(1, 22):
                data_plot = []
                for r in data_dic:
                    data_plot.append(data_dic[r][str(j)])
                plt.plot(list(data_dic.keys())[:end_ind_com],
                         data_plot[:end_ind_com],
                         label=str(j))
            plt.legend()
            plt.savefig(folder_address + "/all_max_idleness.pdf")
            plt.clf()
            print("Plot maxs ok.")
            """
            """
            event_plot_dic = {}
            to_be_reall = []
            reall_plot_dic = {}
            reall_plot_dic = []
            for ind, line in enumerate(data_alloc.iterrows()):
                print(reall_plot_dic)
                if line[1][0] < end_ind_com:
                    if line[1][1] == "robot_3" or line[1][1] == "robot_1":
                        if line[1][2] == 16 and line[1][0] > 100:
                            event_plot_dic[16] = line[1][0]
                            to_be_reall.append(16)
                        if line[1][2] == 19 and line[1][0] > 200:
                            event_plot_dic[19] = line[1][0]
                            to_be_reall.append(19)
                        if line[1][2] == 3 and line[1][0] > 250:
                            event_plot_dic[3] = line[1][0]
                            to_be_reall.append(3)
                    else:
                        if line[1][2] in to_be_reall:
                            reall_plot_dic.append((line[1][2], line[1][0])) # reall_plot_dic[line[1][2]] = line[1][0]
                            to_be_reall.remove(line[1][2])
                    if line[1][2] == 1 and line[1][0] > 300:
                        if not line[1][2] in to_be_reall:
                            event_plot_dic[1] = line[1][0]
                            to_be_reall.append(1)
                    if line[1][2] == 5 and line[1][0] > 350:
                        if not line[1][2] in to_be_reall:
                            event_plot_dic[5] = line[1][0]
                            to_be_reall.append(5)
            """

            if previous_time < end_ind_com:
                end_ind_com = previous_time
            print(i, previous_time, data_max[t])
            max_distrib[i] = data_max[t]
            for t in data_mean:
                tota_mean_com_i1_k1[cr][auction_type][t] += data_mean[t]
            for t in data_max:
                tota_max_com_i1_k1[cr][auction_type][t] += data_max[t]
            for t in data_mean:
                tota_data_com[cr][auction_type][t].append(data_mean[t])
            for t in data_max:
                tota_data_max_com[cr][auction_type][t].append(data_max[t])
            plt.plot(list(data_mean.keys()), list(data_mean.values()), linewidth=.25, label=f"mean")
            """
            last_key = ""
            for key, value in event_plot_dic.items():
                plt.vlines(value, 0, 500, color='r')
                plt.text(value-10, -50, f"{key}", c='r')
            last_key = ""
            for item in reall_plot_dic: # for key, value in reall_plot_dic.items():
                key, value = item
                if not key == last_key:
                    last_key = key
                    plt.vlines(value, 0, 500, color='g')
                    plt.text(value-10, -50, f"{key}", c='g')
            """
            plt.legend()
            plt.savefig(folder_address + "/mean_idleness.pdf")
            plt.clf()
            plt.plot(list(data_max.keys()), list(data_max.values()), linewidth=.25, label=f"max")
            """
            for key, value in event_plot_dic.items():
                plt.vlines(value, 0, 500, color='r')
                plt.text(value-10, -50, f"{key}", c='r')
            last_key = ""
            for item in reall_plot_dic: # for key, value in reall_plot_dic.items():
                key, value = item
                if not key == last_key:
                    last_key = key
                    plt.vlines(value, 0, 500, color='g')
                    plt.text(value-10, -50, f"{key}", c='g')
            """
            plt.legend()
            plt.savefig(folder_address + "/max_idleness.pdf")
            plt.clf()
        for ind, final_max in max_distrib.items():
            plt.scatter(1500, final_max)
        plt.savefig(first_folder + "/final_max_distrib.pdf")
        plt.clf()
        end_ind_com = int(end_ind_com)
        print(end_ind_com)
        end_ind_list.append(end_ind_com)

end_ind_com = min(end_ind_list)
for auction_type in ["ours", "sheng", "no_com"]:
    nb_inst = nb_inst_dic[auction_type]
    plt.plot(list(tota_mean_com_i1_k1[cr][auction_type].keys())[:end_ind_com],
             np.array(list(tota_mean_com_i1_k1[cr][auction_type].values()))[:end_ind_com]/nb_inst,
             label=r'$B = $' + auction_formula[auction_type], color=colors[auction_type])

    """
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
    """

    # Interval de confiance
    std_array = []
    for v in tota_data_com[cr][auction_type].values():
        std_array.append(np.std(list(v)))
    plt.fill_between(np.array(list(tota_mean_com_i1_k1[cr][auction_type].keys())[:end_ind_com]),
                     np.array(list(tota_mean_com_i1_k1[cr][auction_type].values()))[:end_ind_com]/nb_inst+np.array(std_array)[:end_ind_com]*1.96/nb_inst**.5,
                     np.array(list(tota_mean_com_i1_k1[cr][auction_type].values()))[:end_ind_com]/nb_inst-np.array(std_array)[:end_ind_com]*1.96/nb_inst**.5,
                     linewidth=1, alpha=.1, color=colors[auction_type])
    plt.plot(np.array(list(tota_mean_com_i1_k1[cr][auction_type].keys())[:end_ind_com]),
             np.array(list(tota_mean_com_i1_k1[cr][auction_type].values()))[:end_ind_com]/nb_inst+np.array(std_array)[:end_ind_com]*1.96/nb_inst**.5,
             linewidth=1, alpha=.2, color=colors[auction_type])
    plt.plot(np.array(list(tota_mean_com_i1_k1[cr][auction_type].keys())[:end_ind_com]),
             np.array(list(tota_mean_com_i1_k1[cr][auction_type].values()))[:end_ind_com]/nb_inst-np.array(std_array)[:end_ind_com]*1.96/nb_inst**.5,
             linewidth=1, alpha=.2, color=colors[auction_type])
    plt.plot(np.array(list(tota_mean_com_i1_k1[cr][auction_type].keys())[:end_ind_com]),
             np.array(list(tota_mean_com_i1_k1[cr][auction_type].values()))[:end_ind_com]/nb_inst,
             linewidth=1, alpha=1, linestyle="dashed", color=colors[auction_type])


plt.legend()
plt.xlabel("Simulation time (sec.)", fontfamily="serif")
plt.ylabel("Mean idleness (sec.)", fontfamily="serif")
plt.xlim(left=0, right=1500)
plt.ylim(bottom=0, top=1500)
plt.savefig(f"IROS_tests/grid_5x5_r5/mean.pdf")
plt.clf()

for auction_type in ["ours", "sheng", "no_com"]:
    nb_inst = nb_inst_dic[auction_type]
    plt.plot(list(tota_max_com_i1_k1[cr][auction_type].keys())[:end_ind_com],
             np.array(list(tota_max_com_i1_k1[cr][auction_type].values()))[:end_ind_com]/nb_inst,
             label=r'$B = $' + auction_formula[auction_type], color=colors[auction_type])

    """
    # sigma
    std_array = []
    for v in tota_data_com[cr][auction_type].values():
        std_array.append(np.std(list(v)))
    plt.fill_between(np.array(list(tota_max_com_i1_k1[cr][auction_type].keys())[:end_ind_com]),
                     np.array(list(tota_max_com_i1_k1[cr][auction_type].values()))[:end_ind_com]/nb_inst+np.array(std_array)[:end_ind_com],
                     np.array(list(tota_max_com_i1_k1[cr][auction_type].values()))[:end_ind_com]/nb_inst-np.array(std_array)[:end_ind_com],
                     linewidth=1, alpha=.1, color=colors[auction_type])
    plt.plot(np.array(list(tota_max_com_i1_k1[cr][auction_type].keys())[:end_ind_com]),
             np.array(list(tota_max_com_i1_k1[cr][auction_type].values()))[:end_ind_com]/nb_inst+np.array(std_array)[:end_ind_com],
             linewidth=1, alpha=.2, color=colors[auction_type])
    plt.plot(np.array(list(tota_max_com_i1_k1[cr][auction_type].keys())[:end_ind_com]),
             np.array(list(tota_max_com_i1_k1[cr][auction_type].values()))[:end_ind_com]/nb_inst-np.array(std_array)[:end_ind_com],
             linewidth=1, alpha=.2, color=colors[auction_type])
    plt.plot(np.array(list(tota_max_com_i1_k1[cr][auction_type].keys())[:end_ind_com]),
             np.array(list(tota_max_com_i1_k1[cr][auction_type].values()))[:end_ind_com]/nb_inst,
             linewidth=1, alpha=1, linestyle="dashed", color=colors[auction_type])
    """

    # Interval de confiance
    std_array = []
    for v in tota_data_com[cr][auction_type].values():
        std_array.append(np.std(list(v)))
    plt.fill_between(np.array(list(tota_max_com_i1_k1[cr][auction_type].keys())[:end_ind_com]),
                     np.array(list(tota_max_com_i1_k1[cr][auction_type].values()))[:end_ind_com]/nb_inst+np.array(std_array)[:end_ind_com]*1.96/nb_inst**.5,
                     np.array(list(tota_max_com_i1_k1[cr][auction_type].values()))[:end_ind_com]/nb_inst-np.array(std_array)[:end_ind_com]*1.96/nb_inst**.5,
                     linewidth=1, alpha=.1, color=colors[auction_type])
    plt.plot(np.array(list(tota_max_com_i1_k1[cr][auction_type].keys())[:end_ind_com]),
             np.array(list(tota_max_com_i1_k1[cr][auction_type].values()))[:end_ind_com]/nb_inst+np.array(std_array)[:end_ind_com]*1.96/nb_inst**.5,
             linewidth=1, alpha=.2, color=colors[auction_type])
    plt.plot(np.array(list(tota_max_com_i1_k1[cr][auction_type].keys())[:end_ind_com]),
             np.array(list(tota_max_com_i1_k1[cr][auction_type].values()))[:end_ind_com]/nb_inst-np.array(std_array)[:end_ind_com]*1.96/nb_inst**.5,
             linewidth=1, alpha=.2, color=colors[auction_type])
    plt.plot(np.array(list(tota_max_com_i1_k1[cr][auction_type].keys())[:end_ind_com]),
             np.array(list(tota_max_com_i1_k1[cr][auction_type].values()))[:end_ind_com]/nb_inst,
             linewidth=1, alpha=1, linestyle="dashed", color=colors[auction_type])
plt.legend()
plt.xlabel("Simulation time (sec.)", fontfamily="serif")
plt.ylabel("Max. idleness (sec.)", fontfamily="serif")
plt.xlim(left=0, right=1500)
plt.ylim(bottom=0, top=1500)
plt.savefig(f"IROS_tests/grid_5x5_r5/max.pdf")
plt.clf()
print("Plot done.")


tota_data_no_com = {}
tota_data_max_no_com = {}
tota_mean_no_com_i1_k1 = {}
tota_max_no_com_i1_k1 = {}
for cr in [100, 50, 70]:
        print("Starting instances with sheng com and cr = "+str(cr))
        first_folder = "ICRA_tests/simu_icra_obs_"+str(pmul)+"_cr_"+str(cr)+"/no_com"
        end_ind_no_com = 1500
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
	                if round(line[1][1]) < 1500:
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
        plt.plot(np.array(list(tota_mean_com_i1_k1[cr].keys())[:end_ind_com]), np.array(list(tota_mean_com_i1_k1[cr].values()))[:end_ind_com]/nb_inst, linewidth=1, alpha=1, label='Com range = '+str(cr)+', ours term', color=colors[cpt])


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

plt.ylim((0,1500))
plt.xlabel("Temps (sec)")
plt.ylabel("Oisiveté moyenne (sec)")
plt.legend()
plt.savefig("ICRA_tests/mean_idleness_pmul_"+pmul+"_ours_com.pdf")
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

        plt.plot(np.array(list(tota_max_com_i1_k1[cr].keys())[:end_ind_com]), np.array(list(tota_max_com_i1_k1[cr].values()))[:end_ind_com]/nb_inst, linewidth=1, alpha=1, label='Com range = '+str(cr)+', ours term', color=colors[cpt])


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



plt.ylim((0,1500))
plt.xlabel("Temps (sec)")
plt.ylabel("Oisiveté maximale (sec)")
plt.legend()
plt.savefig("ICRA_tests/max_idleness_pmul_"+pmul+"_ours_com.pdf")
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
