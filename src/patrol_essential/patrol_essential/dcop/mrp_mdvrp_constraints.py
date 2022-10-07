"""Defines the constraints for the DCOP MRP inspired of the MDVRP presented in
Léauté, Thomas & Ottens, Brammert & Faltings, Boi. (2010). Ensuring Privacy
through Distributed Computation in Multiple-Depot Vehicle Routing Problems.
"""
import numpy as np
import networkx as nx


def tsp_cost_csts(nb_r, vars, w_poses):
    """Defines the intentional csts to minimize tour length
    :param nb_r: int, number of robots
    :param vars: dic, descriptions for pyDCOP vars
    :param w_poses: iterable, the poses of the waypoints
    """
    tl_csts = {}
    # Intentional constraints for tour length
    for r in range(nb_r):
        tl_csts[f"cost_{r}"] = {
            "type": "intention",
            "function": (
                "if 'solve_tsp_local_search' not in dir():\n"
                "   from python_tsp.heuristics import"
                " solve_tsp_local_search\n"
                "if 'numpy' not in dir(): import numpy\n"
                "v = []\n"
                "w_poses = ["+",".join([str(list(p)) for p in w_poses])+"]\n"
                "vars = [" + ",".join([f"{v}" for v in vars.keys()
                                       if int(v.split('_')[-2]) == r]) + "]\n"
                "for w, wp in enumerate(w_poses):\n"
                "   if vars[w]:\n"
                "       v.append(wp)\n"
                "if len(v) <= 1: return 0\n"
                "r_v = numpy.repeat(v, len(v), axis=0).reshape("
                "len(v), len(v), 2)\n"
                "distance_matrix = numpy.linalg.norm(r_v - numpy.transpose("
                "r_v, axes=(1, 0, 2)), axis=2)\n"
                "if len(v) == 2:\n"
                "   return distance_matrix[0][1] + distance_matrix[1][0]\n"
                "# print(distance_matrix)\n"
                "permutation, distance = "
                "solve_tsp_local_search(distance_matrix)\n"
                "return distance"
                )
            }
    return tl_csts


def tsp_cost_csts_implicit(nb_r, vars, G, w_ids,
                           trivial_wps=[],
                           visit_csts_type="implicit",
                           coms_csts_type="tsp",
                           w_poses=None):
    """Defines the intentional csts to minimize tour length with implicit model
    Note that the method used to compute the TSP impacts the time perf.
    "greedy" converges in O(E**2), while default uses "christofides" (O(E**3))
    :param nb_r: int, number of robots
    :param vars: dic, descriptions for pyDCOP vars
    :param G: navigation graph
    :param w_ids: list of the waypoints concerned by the DCOP
    :coms_csts_type: str from {"tsp", "discwp"}, the
    type of constraints. "tsp" is the basic min tour length.
    "discwp" adds constraints to distribute central waypoints.
    :param w_poses: 2D array of waypoint positions.
    """
    tl_csts = {}
    if coms_csts_type == "tsp":
        for r in range(nb_r):
            tl_csts[f"cost_{r}"] = {
                "type": "intention",
                "function": (
                    "if 'solve_tsp_local_search' not in dir():\n"
                    " from python_tsp.heuristics import solve_tsp_local_search\n"
                    # "if 'solve_tsp_dynamic_programming' not in dir():\n"
                    # " from python_tsp.exact import "
                    # "solve_tsp_dynamic_programming\n"
                    # "import csv\n"
                    # "import time\n"
                    "if 'numpy' not in dir(): import numpy\n"
                    "if 'networkx' not in dir(): import networkx\n"
                    "v = []\n"
                    "w_ids = [" + ",".join([f"{w}" for w in w_ids]) + "]\n"
                    "vars = [" + ",".join([f"{v}" for v in vars.keys()]) + "]\n"
                    "G = networkx.from_edgelist(" + str(nx.to_edgelist(G)) + ")\n"
                    f"v = [wid for w, wid in enumerate(w_ids) if vars[w] == {r}]\n"
                    f"v += [wid for wid in {trivial_wps}]\n"
                    "if len(v) <= 1: return 0\n"
                    # "start = time.time()\n"
                    # "tsp = traveling_salesman_problem(G, 'greedy', nodes=v)\n"
                    # "distance = networkx.classes.function.path_weight("
                    # "G, tsp, 'weight')\n"
                    "table = numpy.zeros((len(v), len(v)))\n"
                    "for ind1, t1 in enumerate(v):\n"
                    "   for ind2, t2 in enumerate(v[ind1+1:]):\n"
                    "       path = networkx.shortest_path(G, source=t1, "
                    "target=t2, weight='weight')\n"
                    "       dist = networkx.path_weight(G, path, "
                    "weight='weight')\n"
                    "       table[ind1, ind1+ind2+1] = dist\n"
                    "       table[ind1+ind2+1, ind1] = dist\n"
                    # "end_sp = time.time() - start\n"
                    "permutation, distance = solve_tsp_local_search(table, "
                    "perturbation_scheme='ps1')\n"  # heuristic solving
                    # ")\n"  # exact solving
                    # "end_tsp = time.time() - start\n"
                    # "with open('TSP_timer_large_improved_caylus_dp.csv', 'a+', "
                    # "newline='') as csvfile:\n"
                    # "    spamwriter = csv.writer(csvfile, delimiter=' ', "
                    # quotechar='|', quoting=csv.QUOTE_MINIMAL)\n"
                    # "    spamwriter.writerow([str(end_sp), str(end_tsp), "
                    # "str(len(v))])\n"
                    "return distance**4"
                    )
                }
    elif coms_csts_type == "discwp":
        for r in range(nb_r):
            tl_csts[f"cost_{r}"] = {
                "type": "intention",
                "function": (
                    "if 'solve_tsp_local_search' not in dir():\n"
                    " from python_tsp.heuristics import solve_tsp_local_search\n"
                    "if 'numpy' not in dir(): import numpy\n"
                    "if 'networkx' not in dir(): import networkx\n"
                    "v = []\n"
                    "w_ids = [" + ",".join([f"{w}" for w in w_ids]) + "]\n"
                    "vars = [" + ",".join([f"{v}" for v in vars.keys()]) + "]\n"
                    "G = networkx.from_edgelist(" + str(nx.to_edgelist(G)) + ")\n"
                    f"v = [wid for w, wid in enumerate(w_ids) if vars[w] == {r}]\n"
                    f"v += [wid for wid in {trivial_wps}]\n"
                    "if len(v) <= 1: return 0\n"
                    "table = numpy.zeros((len(v), len(v)))\n"
                    "for ind1, t1 in enumerate(v):\n"
                    "   for ind2, t2 in enumerate(v[ind1+1:]):\n"
                    "       path = networkx.shortest_path(G, source=t1, "
                    "target=t2, weight='weight')\n"
                    "       dist = networkx.path_weight(G, path, "
                    "weight='weight')\n"
                    "       table[ind1, ind1+ind2+1] = dist\n"
                    "       table[ind1+ind2+1, ind1] = dist\n"
                    "permutation, distance = solve_tsp_local_search(table, "
                    "perturbation_scheme='ps1')\n"
                    "tour_poses = numpy.empty((len(v), 2))\n"
                    f"w_poses = [" + ",".join([f"{p}" for p in w_poses.values()]) + "]\n"
                    "for i, w in enumerate(v):\n"
                    "   tour_poses[i] = w_poses[int(w)]\n"
                    "grav_center = numpy.mean(tour_poses, axis=0)\n"
                    "central_waypoint = None\n"
                    "min_dist = 1e6\n"
                    "for w, p in enumerate(w_poses):\n"
                    "   dist = numpy.linalg.norm(grav_center-numpy.array(p))\n"
                    "   if dist < min_dist:\n"
                    "       min_dist = dist\n"
                    "       central_waypoint = w\n"
                    f"penalty = 0\n"
                    "for i, w in enumerate(w_ids):\n"
                    "   if w == central_waypoint:\n"
                    f"       if vars[i] == {r}:\n"
                    "           penalty = 1e9\n"
                    "return distance**4 + penalty"
                    )
                }
    return tl_csts


def tsp_max_csts_implicit(nb_r, vars, w_poses):
    """Defines the intentional csts to minimize tour length with implicit model
    :param nb_r: int, number of robots
    :param vars: dic, descriptions for pyDCOP vars
    :param w_poses: iterable, the poses of the waypoints
    """
    tl_csts = {}
    for r in range(nb_r):
        tl_csts["cost"] = {
            "type": "intention",
            "function": (
                "if 'solve_tsp_local_search' not in dir():\n"
                "   from python_tsp.heuristics import"
                " solve_tsp_local_search\n"
                "if 'numpy' not in dir(): import numpy\n"
                "v = []\n"
                "w_poses = ["+",".join([str(list(p)) for p in w_poses])+"]\n"
                "vars = [" + ",".join([f"{v}" for v in vars.keys()]) + "]\n"
                "m = -1\n"
                f"for r in range({nb_r}):\n"
                "   for w, wp in enumerate(w_poses):\n"
                "      if vars[w] == r:\n"
                "           v.append(wp)\n"
                "   if len(v) <= 1:\n"
                "       m = max([0, m])\n"
                "       continue\n"
                "   r_v = numpy.repeat(v, len(v), axis=0).reshape("
                "len(v), len(v), 2)\n"
                "   distance_matrix = numpy.linalg.norm(r_v - numpy.transpose("
                "r_v, axes=(1, 0, 2)), axis=2)\n"
                "   if len(v) == 2:\n"
                "       m = max([m, distance_matrix[0][1] + "
                "distance_matrix[1][0]])\n"
                "       continue\n"
                "   permutation, distance = "
                "   solve_tsp_local_search(distance_matrix)\n"
                "   m = max([m, distance])\n"
                "return m"
                )
            }
    return tl_csts


def visit_csts(nb_w, vars):
    """Defines the pseudo_hard csts to ensure that each waypoint is visited
    :param nb_w: int, number of waypoints
    :param vars: dic, descriptions for pyDCOP vars
    """
    vt_csts = {}
    # Each waypoint must be visited at least once
    for w in range(nb_w):
        vt_csts[f"visited_{w}"] = {
            "type": "intention",
            "function": ("0 if sum(["
                         + ",".join([f"{v}" for v in vars.keys()
                                     if int(v.split('_')[-1]) == w])
                         + "]) == 1 else 1000")
            }
    return vt_csts


def visit_csts_extensional(nb_w, nb_r, vars):
    """Defines the pseudo_hard csts to ensure that each waypoint is visited
    :param nb_w: int, number of waypoints
    :param vars: dic, descriptions for pyDCOP vars
    """
    vt_csts = {}
    Id = np.identity(nb_r, np.uint8)
    values = " | ".join([str(i) for i in Id]).replace("[", "").replace("]", "")
    # Each waypoint must be visited at least once
    for w in range(nb_w):
        vt_csts[f"visited_{w}"] = {
            "type": "extensional",
            "variables": [v for v in vars.keys()
                          if int(v.split('_')[-1]) == w],
            "values": {-1000: values}
            }
        """
        ("0 if sum(["
                     + ",".join([f"{v}" for v in vars.keys()
                                 if int(v.split('_')[-1]) == w])
                     + "]) == 1 else 1000")
        """
    return vt_csts
