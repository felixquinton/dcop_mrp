"""Defines the Tkinter class that displays robots, tasks, etc...
"""
import tkinter as tk
import numpy as np


class MissionDisplayV2(tk.Canvas):
    """
    Defines the Tkinter object that displays robots, tasks, etc...
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.robot_dic = {}
        self.com_range_dic = {}
        self.color_dic = {}
        self.text_dic = {}
        self.com_range_circles = {}
        self.edges_dic = {}

        # Lines for the communication network
        self.cn_lines = []

    def init_display_from_spec_data(self, mission_spec_data, robots_spec_data):
        """
        Initializes the mission display with proper nodes and edges.
        :param mission_spec_data: mission specs as parsed from utils functions.
        :param robots_spec_data: robot specs as parsed from utils functions.
        """
        edges = {}
        for waypoint, waypoint_data in mission_spec_data.items():
            waypoint_id = waypoint.split('_')[-1]
            waypoint_pos = (waypoint_data["pos"][0],
                            480 - waypoint_data["pos"][1])
            for neighbor in waypoint_data["neighbors"]:
                if not edges.get((neighbor, waypoint), False):
                    edges[(waypoint, neighbor)] = True
                    neighbor_pos = mission_spec_data[neighbor]["pos"]
                    neighbor_pos = (
                        neighbor_pos[0],
                        480 - neighbor_pos[1])
                    self.create_edge(
                        waypoint_pos, neighbor_pos, waypoint, neighbor)
            self.create_waypoint(waypoint_pos, waypoint_id)
        for robot_id in robots_spec_data.keys():
            pass  # self.create_robot(robot_id)

    def update_display(self):
        """
        Updates the display. Draws items if checked in the display_bar.
        """
        self.update()
        for item, draw in self.robot_dic.items():
            self.itemconfig(draw,
                            fill=self.color_dic[item],
                            outline='black')
            self.itemconfig(self.text_dic[item], fill="black")

    def update_positions(self, item, pos):
        """
        Updates the robots' positions.
        :param item: dict, with "drawable" and "text" keys corresponding to two
          Tkinter drawables.
        :param pos: list of float with at least 2 indices, the updated position
          of the object. If list is more than 2 indices, subsequent indices
          will be ignored.
        """
        x0, y0, rx0, ry0 = self.coords(item["drawable"])
        width = rx0-x0
        height = ry0-y0
        dx = pos[0]-x0
        dy = pos[1]-y0
        self.move(item["drawable"], dx-width/2, dy-height/2)
        self.move(item["text"], dx-width/2, dy-height/2)
        self.move(item["com_range"], dx-width/2, dy-height/2)

    def update_com_network(self, com_graph):
        """
        Updates the team communication network.
        :param com_graph: communication network as an nx graph.
        """
        try:
            for line in self.cn_lines:
                self.delete(line)
            for edge in com_graph.edges:
                robot_source = self.robot_dic["robot_"+edge[0]]
                robot_target = self.robot_dic["robot_"+edge[1]]
                x0, y0, *_ = self.coords(robot_source)
                x1, y1, *_ = self.coords(robot_target)
                self.cn_lines.append(
                    self.create_line(x0, y0, x1, y1, dash=(3, 5)))
        except KeyError:
            pass

    def create_waypoint(self, pos, waypoint_id=""):
        """
        Initialize a waypoint of the navigation graph.
        :param pos:  list of float with exactly 2 indices, the position of the
          waypoint in a 2D plane.
        :param waypoint_id: string, the name of the waypoint.
        """
        self._create_circle(*pos, 10, fill="white", outline="blue")
        self.create_text(*pos, text=waypoint_id)
        self.update()

    def create_edge(self, p1, p2, w1, w2, cost="", **kwargs):
        """
        Initialize an edge of the navigation graph.
        :param w1: list of float with exactly 2 indices, the position of the
          first waypoint in a 2D plane.
        :param w2: list of float with exactly 2 indices, the position of the
          second waypoint in a 2D plane.
        :param cost: string, the cost associated with the edge.
        """
        line = self.create_line(*p1, *p2, **kwargs)
        self.edges_dic[(w1, w2)] = line
        self.update()

    def create_robot(self, robot_id, com_range=0, robot_colors={}):
        """
        Initialize a robot's display objects.
        :param robot_id: int, the id of the robot to create on the display.
        :param robot_colors: dict, used to set robots' colors. Keys are robot
          ids, values are matplotlib compatible colors.
        :return item: dict, with keys:
          "drawable": a rectangle that represent the robot on the main canvas.
          "text": a text that move along the drawable.
        """
        robot_display = {}
        try:
            color = robot_colors[robot_id]
            self.color_dic[robot_id] = robot_colors[robot_id]
        except KeyError:
            color = self._generate_random_color()
            self.color_dic[robot_id] = color
        robot_display["com_range"] = self._create_circle(
            0, 0, com_range, fill=color, stipple='gray12')
        self.com_range_circles[robot_id] = robot_display["com_range"]
        robot_display["drawable"] = self.create_rectangle(
            0, 0, 20, 10, fill=color, outline="black")
        self.robot_dic[robot_id] = robot_display["drawable"]
        robot_display["text"] = self.create_text((0, 10), text=robot_id)
        self.text_dic[robot_id] = robot_display["text"]
        return robot_display

    def edit_edge(self, p1, p2, w1, w2, forbidden_types):
        str_w1 = f"waypoint_{w1}"
        str_w2 = f"waypoint_{w2}"
        p1 = (p1[0], self.winfo_height() - p1[1])
        p2 = (p2[0], self.winfo_height() - p2[1])
        if (str_w1, str_w2) in self.edges_dic.keys():
            self.delete(self.edges_dic[(str_w1, str_w2)])
            if forbidden_types[0] == "UGV":
                self.create_edge(p1, p2, str_w1, str_w2,
                                 dash=(1, 1), fill=('red'))
            else:
                self.create_edge(p1, p2, str_w1, str_w2,
                                 dash=(1, 1), fill=('black'))
        elif (str_w2, str_w1) in self.edges_dic.keys():
            self.delete(self.edges_dic[(str_w2, str_w1)])
            if forbidden_types[0] == "UGV":
                self.create_edge(p2, p1, str_w2, str_w1,
                                 dash=(1, 1), fill=('red'))
            else:
                self.create_edge(p2, p1, str_w2, str_w1,
                                 dash=(1, 1), fill=('black'))

    def _create_circle(self, x, y, r, **kwargs):
        return self.create_oval(x-r, y-r, x+r, y+r, **kwargs)

    def _generate_random_color(self):
        """
        Generate a random color for a each robot.
        TODO : improve this so that no two robots are of the same color.
        """
        rgbl = np.random.randint(0, 2, 3)
        while rgbl.sum() == 3:
            rgbl = np.random.randint(0, 2, 3)
        return "#%02x%02x%02x" % tuple(rgbl*255)
