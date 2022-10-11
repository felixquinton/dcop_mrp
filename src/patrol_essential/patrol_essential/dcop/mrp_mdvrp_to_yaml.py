"""This module builds upon the constraints defined in mrp_mdvrp_constraints.py
in order to write down a pyDCOP instance YAML file.
"""

import yaml
import numpy as np
import networkx as nx
from patrol_essential.dcop.mrp_mdvrp_constraints import (
    visit_csts, visit_csts_extensional, tsp_cost_csts_implicit)


def write_dcop_yaml(r_ids, w_ids, r_sen, w_req, G,
                    trivial_assignments,
                    algorithm="mgm2",
                    visit_csts_type="implicit",
                    instance_file_name="./test_mrpp_mdvrp_inst.yaml",
                    distribution_file_name="./test_mrpp_mdvrp_"
                    "distribution.yaml"):

    trivial_wps = list(trivial_assignments.keys())
    dcop_dic = create_dcop(r_ids, w_ids, r_sen, w_req, G, trivial_wps,
                           algorithm=algorithm,
                           visit_csts_type=visit_csts_type)
    print("DCOP created")

    try:
        print("trying to write at ", instance_file_name)
        with open(instance_file_name, 'w') as file:
            yaml.dump(dcop_dic, file, default_flow_style=False)
        print("DCOP written at ", instance_file_name)
    except Exception as e:
        raise e

    distrib = create_distrib(
        dcop_dic["agents"], dcop_dic["variables"],
        visit_csts_type, algorithm=algorithm)

    distrib_dic = {"distribution": distrib}

    try:
        with open(distribution_file_name, 'w') as file:
            yaml.dump(distrib_dic, file, default_flow_style=False)
    except Exception as e:
        raise e


def create_dcop(r_ids, w_ids, r_sen, w_req, G, trivial_wps,
                algorithm="mgm2", visit_csts_type="implicit",
                coms_csts_type="tsp"):
    nb_w = len(w_ids)
    nb_r = len(r_ids)

    domains = create_domains(w_ids, r_ids, w_req, r_sen,
                             visit_csts_type=visit_csts_type)
    print("Domains created", domains, w_ids, w_req)
    vars = create_vars(w_ids, nb_r, visit_csts_type, domains)
    print("Vars created")
    csts = create_constraints_flat(
       nb_w, nb_r, vars, visit_csts_type,
       G, w_ids, trivial_wps, coms_csts_type)
    print("Constraints created")
    agents = create_agents(nb_r)
    print("Agents created")

    res_dic = {"name": "test_mrpp",
               "objective": "min",
               "domains": domains,
               "variables": vars,
               "constraints": csts,
               "agents": agents}

    return res_dic


def create_vars(w_ids, nb_r, visit_csts_type, domains):
    """Boolean var x_r^w is 1 if robot i survey waypoint w, 0 otherwise.
    Or, if visit_csts_type == "implicit", var x^w indicated the id of the robot
    surveying w.
    :param nb_w: int, number of waypoints of the mission
    :param nb_r: int, number of robots of the team
    :visit_csts_type: str from {"intentional", "extensional", "implicit"}, the
    type of constraints expressing that waypoints must be visited exactly once.
    """
    vars = {}
    if visit_csts_type == "implicit":
        for w_ind, w in enumerate(w_ids):
            vars[f"x_{w_ind}"] = {"domain": f'd_{w}',
                                  "initial_value": int(np.random.choice(
                                      list(domains[f"d_{w}"]["values"])))}
    else:
        for w in range(len(w_ids)):
            for r in range(nb_r):
                vars[f"x_{r}_{w}"] = {"domain": 'd1', "initial_value": 0}
    return vars


def create_constraints_flat(nb_w, nb_r, vars, visit_csts_type, G,
                            w_ids, trivial_wps, coms_csts_type):
    """Builds the constraints.
    :param nb_w: int, number of waypoints of the mission
    :param nb_r: int, number of robots of the team
    :param vars: dic built using create_vars, describes the DCOP variables
    :coms_csts_type: str from {"tsp", "discwp"}, the
    type of constraints. "tsp" is the basic min tour length.
    "discwp" adds constraints to distribute central waypoints.
    :param path_lengths: dict, the shortest_paths in the navigation_graph
    :visit_csts_type: str from {"intentional", "extensional", "implicit"}, the
    type of constraints expressing that waypoints must be visited exactly once.
    """
    try:
        w_poses = nx.get_node_attributes(G, 'pos')
        csts = tsp_cost_csts_implicit(
            nb_r, vars, G, w_ids, trivial_wps,
            visit_csts_type, coms_csts_type, w_poses)
    except Exception:
        raise ValueError("Error while creating DCOP tsp cost constraints.")
    return csts


def create_agents(nb_r):
    """Defines the agents.
    :param nb_r: int, number of robots of the team
    """
    agents = {}
    for r in range(nb_r):
        agents[f'r_{r}'] = {'capacity': 1000}
    return agents


def create_distrib(agents, vars, visit_csts_type, algorithm="mgm2"):
    """Builds the distribution yaml file to indicate which agents does what.
    :param agents: dic, generated with create_agents()
    :param vars: dic, generated with create_vars()
    :visit_csts_type: str from {"intentional", "extensional", "implicit"}, the
    type of constraints expressing that waypoints must be visited exactly once.
    :param algorithm: str, the pyDCOP algorithm that will be used.
    """
    res = {}
    for a in agents:
        if visit_csts_type == "implicit":
            res[a] = [v for v in vars
                      if (int(v.split('_')[-1]) + 1) % len(
                          agents) == int(a.split('_')[-1])]
        else:
            res[a] = [v for v in vars if v.split('_')[-2] == a.split('_')[-1]]
    if algorithm in ["maxsum"]:
        # Need to specify the distribution of the constraints nodes
        for a in agents:
            res[a].append(f"cost_{int(a.split('_')[-1])}")
    return res


def create_domains(w_ids, r_ids, w_req, r_sen, visit_csts_type="implicit"):
    """Defines the variables domains.
    :param w_ids: list of int, waypoint ids
    :param r_ids: list of int, robot ids
    :param w_req: dic, keys are waypoint ids, values are sensor requirements
    :param r_sen: dic, keys are robots ids, values are avalable sensors
    :visit_csts_type: str from {"intentional", "extensional", "implicit"}, the
    type of constraints expressing that waypoints must be visited exactly once.
    """
    if visit_csts_type == "implicit":
        res = {f'd_{w}': {"values": [r_ind for r_ind, r in enumerate(r_ids)
                                     if set(w_req[w]) & set(r_sen[r])]
                          }
               for w in w_ids}
    else:
        # TODO: Sensor match for non implicit models
        res = {"d1": {"values": [0, 1]}}
    return res
