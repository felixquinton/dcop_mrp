"""Defines the Tkinter class that displays robots, tasks, etc...
"""

from caf_interface.mission_display import MissionDisplay as BaseDisplay


class MissionDisplay(BaseDisplay):
    """
    Defines the Tkinter object that displays robots, tasks, etc...
    """

    def __init__(self, *args, **kwargs):
        self.edges_dic = {}
        super().__init__(*args, **kwargs)

    def create_edge(self, p1, p2, w1, w2, cost="", **kwargs):
        """
        Initialize an edge of the navigation graph.
        :param w1: list of float with exactly 2 indices, the position of the
          first waypoint in a 2D plane.
        :param w2: list of float with exactly 2 indices, the position of the
          second waypoint in a 2D plane.
        :param cost: string, the cost associated with the edge.
        """
        line = self.canvas.create_line(*p1, *p2, **kwargs)
        self.edges_dic[(w1, w2)] = line
        self.update()

    def edit_edge(self, p1, p2, w1, w2, forbidden_types):
        print(w1, w2, self.edges_dic, self.edges_dic.keys())
        str_w1 = f"waypoint_{w1}"
        str_w2 = f"waypoint_{w2}"
        p1 = (p1[0], self.canvas.winfo_height() - p1[1])
        p2 = (p2[0], self.canvas.winfo_height() - p2[1])
        if (str_w1, str_w2) in self.edges_dic.keys():
            print("Inside first if")
            self.canvas.delete(self.edges_dic[(str_w1, str_w2)])
            if forbidden_types[0] == "UGV":
                self.create_edge(p1, p2, str_w1, str_w2,
                                 dash=(1, 1), fill=('red'))
            else:
                self.create_edge(p1, p2, str_w1, str_w2,
                                 dash=(1, 1), fill=('black'))
        elif (str_w2, str_w1) in self.edges_dic.keys():
            print("Inside second if")
            self.canvas.delete(self.edges_dic[(str_w2, str_w1)])
            if forbidden_types[0] == "UGV":
                self.create_edge(p2, p1, str_w2, str_w1,
                                 dash=(1, 1), fill=('red'))
            else:
                self.create_edge(p2, p1, str_w2, str_w1,
                                 dash=(1, 1), fill=('black'))
