import numpy as np
import networkx as nx
import json
import subprocess

from python_tsp.exact import solve_tsp_dynamic_programming
from patrol_essential.dcop.mrp_mdvrp_to_yaml import write_dcop_yaml


def find_trivial_assignments(r_ids, w_ids, r_sen, w_req):
    try:
        able_robots = {w: [r_ind for r_ind, r in enumerate(r_ids)
                           if set(req) & set(r_sen[int(r)-1])]
                       for w, req in w_req.items()}
    except Exception:
        raise

    trivial = {}
    unfeasible = []
    all_trivial = True
    print(f'Able_robots: {able_robots}')
    for w, rs in able_robots.items():
        w_id = list(w_ids).index(w)

        if not rs:
            unfeasible.append(w_id)

        elif len(rs) == 1:
            trivial[w_id] = rs[0]  # index of the robot in r_ids

        else:
            all_trivial = False

    print("UNFEASIBLE", unfeasible)

    return {"all_trivial": all_trivial,
            "trivial_assignments": trivial,
            "unfeasible_tasks": unfeasible}


def solve_dcop(r_ids, w_ids, r_sen, w_req, w_poses, G,
               alg="mgm2",
               instance_file_name="mission_logs/yaml_dcops/"
               "test_mrpp_mdvrp_inst.yaml",
               distribution_file_name="mission_logs/yaml_dcops/"
               "test_mrpp_mdvrp_distribution.yaml",
               sim_node=None):
    """Solves the MRP DCOP with pyDCOP. Returns a dic corresponding to pyDCOP
    output.
    :return res: dic, pyDCOP's output
    """
    nb_w = len(w_ids)
    w_req_keys = list(w_req.keys())
    for w in w_req_keys:
        if w not in w_ids:
            del w_req[w]
    sim_node.get_logger().info(f"After deletion {w_req}")
    fta = find_trivial_assignments(r_ids, w_ids, r_sen, w_req)
    sim_node.get_logger().info("After trivial search")
    is_trivial = fta["all_trivial"]
    pre_assignment = fta["trivial_assignments"]
    pre_assigned_tasks = {w_ids[w]: r for w, r in pre_assignment.items()}
    unfeasible_tasks = fta["unfeasible_tasks"]
    sim_node.get_logger().info(
        f"Trivial: {pre_assignment}, pre assigned tasks {pre_assigned_tasks}")
    # sim_node.get_logger().info(f"Before unfeasible {nb_w}, {w_req}")
    for w in unfeasible_tasks:
        sim_node.get_logger().info(
            f"The unfeasible waypoint is at index {w} and is {w_ids[w]}")
        sim_node.get_logger().info(f"At this point w_ids is {w_ids}")
        nb_w -= 1
        # del w_req[list(w_req.keys())[w]]
        # w_poses = np.delete(w_poses, w, axis=0)
    # sim_node.get_logger().info(f"After unfeasible {nb_w}, {w_req}")
    for w_ind in pre_assignment:
        nb_w -= 1
        # del w_req[list(w_req.keys())[w_ind]]
        # w_poses = np.delete(w_poses, w_ind, axis=0)
    sim_node.get_logger().info(f"PREASSIGNMENT KEYS: {pre_assignment.keys()}")
    w_ids = np.delete(w_ids, unfeasible_tasks + list(pre_assignment.keys()))
    sim_node.get_logger().info(f"w_ids after preassignment and unfeasibles: {w_ids}")
    feasible_w_req = {i: req for i, req in w_req.items()}
    if is_trivial:
        assignment_dic = json.dumps(
            {"assignment": {f'x_{w}': r for w, r in pre_assignment.items()}})
        sim_node.get_logger().info(f"Assignment dic: {assignment_dic}.")
        return (json.dumps(
            # {"assignment": {f'x_{w}': r for w, r in pre_assignment.items()}}),
            {"assignment": {}}),
            True, unfeasible_tasks, pre_assignment, pre_assigned_tasks)
    try:
        int_r_ids = [int(r)-1 for r in r_ids]
        sim_node.get_logger().info(f"JUSTE AVANT BUG {int_r_ids}")
        write_dcop_yaml(
            int_r_ids, w_ids, r_sen, feasible_w_req, G,
            pre_assigned_tasks,
            algorithm=alg,
            instance_file_name=instance_file_name,
            distribution_file_name=distribution_file_name)
        sim_node.get_logger().info("ITS ALL RIGHT ACTUALLY !")
    except Exception as e:
        raise e
    sim_node.get_logger().info("Starting DCOP")
    # stop_cycle:2 is already not converging in 6 minutes...
    cmd = (f"pydcop -t 30 solve  -a {alg} --algo_params stop_cycle:2 "
           f"-d {distribution_file_name} -m thread {instance_file_name}")

    output = subprocess.run(cmd, shell=True, capture_output=True)
    sim_node.get_logger().info("Finished DCOP")
    return (output, False, unfeasible_tasks,
            pre_assignment, pre_assigned_tasks)


def get_assignment_from_pydcop(sim_node, stdout, subteam_ids, waypoint_ids,
                               pre_assigned_tasks,
                               trivial=False):
    """Parses the output from a pyDCOP call in order to obtain the
    assignment.
    :param sim_node: ROS2 Node.
    :param sdtout: byte or str, shell output from a call to pyDCOP.
    :param subteam_ids: set, the ids of the robots conserned by the DCOP.
    :return assigment: dic, with keys being robot ids and values being
    lists of waypoint ids.
    """
    print("ici", trivial)
    try:
        if not trivial:
            dic_output = json.loads(
                stdout.stdout.decode("utf-8").replace(
                    "\n", "").replace(" ", ""))
        else:
            dic_output = json.loads(stdout)
        print("decodage")
        solution = dic_output["assignment"]
        sim_node.get_logger().info(f"Solution DCOP: {solution, subteam_ids}")
        assignment = {}
        sim_node.get_logger().info(f"Robots de la sous-équipe: {subteam_ids}")
        sim_node.get_logger().info(f"Waypoints alloués: {waypoint_ids}")
        for w, r in solution.items():
            w_ind = int(w.split('_')[-1])
            # sim_node.get_logger().info(f"Data before (w, w_ind, r, subteam_ids, waypoint_ids, assignment): {w, waypoint_ids[w_ind], r, subteam_ids, waypoint_ids, assignment}")
            r_id = int(subteam_ids[r])
            # sim_node.get_logger().info(f"Affectation (w, r): {waypoint_ids[w_ind], r_id}")
            assignment[r_id] = assignment.get(
                r_id, []) + [waypoint_ids[w_ind]]
        print("là")
        print("comment", pre_assigned_tasks)
        for w, r in pre_assigned_tasks.items():
            r_id = int(subteam_ids[r])
            sim_node.get_logger().info(f"Préallocation (w, r): {w, r_id}")
            assignment[r_id] = assignment.get(r_id, []) + [w]
        sim_node.get_logger().info(f"get_assignment_from_pydcop {assignment}")
    except json.decoder.JSONDecodeError as e:
        sim_node.get_logger().error("It appears that pyDCOP did not return a"
                                    f"json str: {stdout}")
        raise e
    except KeyError as e:
        sim_node.get_logger().error("It appears that 'assignment' was not in"
                                    f"dic_output: {dic_output}")
        raise e
    return assignment


def compute_tours(graph, assignment, sim_node):
    """Compute TSP optimal tours, given an assignment.
    :param graph: nx Graph, must have a 'pos' attributes on each node.
    :param assignment: dic, with keys being robot ids and values being
    lists of waypoint ids.
    :return tours: dic, with keys being robot ids and values being ordered
    list of waypoint ids corresponding to each robot's TSP tour.
    """
    tours = {}
    for r, ws in assignment.items():
        # if there are less than 3 waypoints in the tour, order doesn't matter.
        if len(ws) <= 2:
            tours[r] = ws
            sim_node.get_logger().info(f"cond tours {tours[r]} ws {ws}")
            continue
        # Finding the tour's waypoints' poses and indices.
        wps = [p for n, p in nx.get_node_attributes(
            graph, 'pos').items() if n in ws]
        w_inds = np.array([n for n in graph.nodes if n in ws])
        sim_node.get_logger().info(f"w_inds {w_inds} ws {ws}")
        # Building the distance matrix.
        r_wps = np.repeat(
            wps, len(wps), axis=0).reshape(len(wps), len(wps), 2)
        distance_matrix = np.linalg.norm(r_wps - np.transpose(
            r_wps, axes=(1, 0, 2)), axis=2)
        # Computing TSP tour.
        sim_node.get_logger().info("Starting to compute TSP.")
        tsp_tour, _ = solve_tsp_dynamic_programming(distance_matrix)
        tours[r] = w_inds[tsp_tour]
        sim_node.get_logger().info(
            f"tours[r] {tours[r]}, tsp_tours {tsp_tour}")
    return tours
