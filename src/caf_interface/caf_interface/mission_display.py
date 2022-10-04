"""Defines the Tkinter class that displays robots, tasks, etc...
"""

import tkinter as tk
import numpy as np


class MissionDisplay(tk.Tk):
    """
    Defines the Tkinter object that displays robots, tasks, etc...
    """

    def __init__(self, *args, **kwargs):
        self.robot_dic = {}
        self.com_range_dic = {}
        self.color_dic = {}
        self.text_dic = {}
        self.com_range_circles = {}

        # Initializes a window
        tk.Tk.__init__(self, *args, **kwargs)

        self.geometry("800x600")

        w = 600
        h = 400

        # Sets up the canvas
        self.canvas = tk.Canvas(self, width=w, heigh=h, bg="white",
                                highlightthickness=1,
                                highlightbackground="black")
        self.canvas.pack(pady=20)
        self.canvas.place(x=20, y=20)

        # Sets up the energy bar
        w_energy = 140
        h_energy = 400
        self.display_bar = tk.Canvas(self, width=w_energy, heigh=h_energy,
                                     bg="white", highlightthickness=1,
                                     highlightbackground="black")
        self.display_bar.pack(pady=20)
        self.display_bar.place(x=780-w_energy, y=20)
        self.display_bar.create_text((30, 10), text="Items")
        self.robot_display_var = {}
        self.com_range_display_var = {}

        # Sets up the display bar
        w_energy = 600
        h_energy = 100
        self.energy_bar = tk.Canvas(self, width=w_energy, heigh=h_energy,
                                    bg="white", highlightthickness=1,
                                    highlightbackground="black")
        self.energy_bar.create_circle = self._create_circle
        self.energy_bar.create_circle_arc = self._create_circle_arc
        self.energy_bar.pack(pady=20)
        self.energy_bar.place(x=20, y=h+40)
        self.energy_bar.create_text((30, 10), text="Energy")

        # Lines for the communication network
        self.cn_lines = []

    def update_display(self):
        """
        Updates the display. Draws items if checked in the display_bar.
        """
        self.update()
        for item, draw in self.robot_dic.items():
            if (self.com_range_display_var[item] is not None
                    and self.com_range_display_var[item].get() == 0):
                self.canvas.itemconfig(self.com_range_circles[item],
                                       fill='', outline='')
            elif (self.com_range_display_var[item] is not None
                    and self.com_range_display_var[item].get() == 1):
                self.canvas.itemconfig(self.com_range_circles[item],
                                       fill=self.color_dic[item],
                                       outline='black', stipple='gray12')
            if (self.robot_display_var[item] is not None
                    and self.robot_display_var[item].get() == 0):
                self.canvas.itemconfig(draw, fill='', outline='')
                self.canvas.itemconfig(self.text_dic[item], fill="")
            elif (self.robot_display_var[item] is not None
                    and self.robot_display_var[item].get() == 1):
                self.canvas.itemconfig(draw,
                                       fill=self.color_dic[item],
                                       outline='black')
                self.canvas.itemconfig(self.text_dic[item], fill="black")

    def update_positions(self, item, pos):
        """
        Updates the robots' positions.
        :param item: dict, with "drawable" and "text" keys corresponding to two
          Tkinter drawables.
        :param pos: list of float with at least 2 indices, the updated position
          of the object. If list is more than 2 indices, subsequent indices
          will be ignored.
        """
        x0, y0, rx0, ry0 = self.canvas.coords(item["drawable"])
        width = rx0-x0
        height = ry0-y0
        dx = pos[0]-x0
        dy = pos[1]-y0
        self.canvas.move(item["drawable"], dx-width/2, dy-height/2)
        self.canvas.move(item["text"], dx-width/2, dy-height/2)
        self.canvas.move(item["com_range"], dx-width/2, dy-height/2)

    def update_energy(self, robot_id, item, energy):
        """
        Updates the robots' energy info.
        :param robot_id: int, the id of the robot whose energy is updated.
        :param item: dict, with "drawable" and "text" keys corresponding to two
          Tkinter drawables.
        :param energy: float, the updated value of the energy the robot.
        """
        item["energy"].configure(
            text=str(robot_id) + " has " + str(round(energy, 3))
            + " remaining energy.")

    def update_com_network(self, com_graph):
        """
        Updates the team communication network.
        :param com_graph: communication network as an nx graph.
        """
        for line in self.cn_lines:
            self.canvas.delete(line)
        for edge in com_graph.edges:
            robot_source = self.robot_dic["robot_"+edge[0]]
            robot_target = self.robot_dic["robot_"+edge[1]]
            x0, y0, *_ = self.canvas.coords(robot_source)
            x1, y1, *_ = self.canvas.coords(robot_target)
            self.cn_lines.append(
                self.canvas.create_line(x0, y0, x1, y1, dash=(3, 5)))

    def create_robot(self, robot_id, com_range=0, robot_colors={}):
        """
        Initialize a robot's display objects.
        :param robot_id: int, the id of the robot to create on the display.
        :param robot_colors: dict, used to set robots' colors. Keys are robot
          ids, values are matplotlib compatible colors.
        :return item: dict, with keys:
          "drawable": a rectangle that represent the robot on the main canvas.
          "text": a text that move along the drawable.
          "energy": a label that displays the robot's energy in the energy_bar.
          "is_displayed_box": a checkbox that controls the display of the
          robot's drawable and text on the main canvas.
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
        self.com_range_display_var[robot_id] = tk.IntVar()
        self.com_range_display_var[robot_id].set(1)
        robot_display["is_displayed_com_range_box"] = tk.Checkbutton(
            self.display_bar, text="com range "+str(robot_id),
            variable=self.com_range_display_var[robot_id],
            command=self.on_check_robot_display(
                self.com_range_display_var[robot_id]))
        robot_display["is_displayed_com_range_box"].pack()
        robot_display["drawable"] = self.canvas.create_rectangle(
            0, 0, 20, 10, fill=color, outline="black")
        self.robot_dic[robot_id] = robot_display["drawable"]
        robot_display["text"] = self.canvas.create_text((0, 10), text=robot_id)
        self.text_dic[robot_id] = robot_display["text"]
        robot_display["energy"] = tk.Label(
            self.energy_bar, text=(str(robot_id)+" has "+str()
                                   + " remaining energy."))
        robot_display["energy"].pack()
        self.robot_display_var[robot_id] = tk.IntVar()
        self.robot_display_var[robot_id].set(0)
        robot_display["is_displayed_box"] = tk.Checkbutton(
            self.display_bar, text=str(robot_id),
            variable=self.robot_display_var[robot_id],
            command=self.on_check_robot_display(
                self.robot_display_var[robot_id]))
        robot_display["is_displayed_box"].pack()
        return robot_display

    def create_waypoint(self, pos, waypoint_id=""):
        """
        Initialize a waypoint of the navigation graph.
        :param pos:  list of float with exactly 2 indices, the position of the
          waypoint in a 2D plane.
        :param waypoint_id: string, the name of the waypoint.
        """
        self._create_circle(*pos, 10, fill="white", outline="blue")
        self.canvas.create_text(*pos, text=waypoint_id)
        self.update()

    def create_edge(self, w1, w2, cost=""):
        """
        Initialize an edge of the navigation graph.
        :param w1: list of float with exactly 2 indices, the position of the
          first waypoint in a 2D plane.
        :param w2: list of float with exactly 2 indices, the position of the
          second waypoint in a 2D plane.
        :param cost: string, the cost associated with the edge.
        """
        self.canvas.create_line(*w1, *w2)
        self.update()

    def _generate_random_color(self):
        """
        Generate a random color for a each robot.
        TODO : improve this so that no two robots are of the same color.
        """
        rgbl = np.random.randint(0, 2, 3)
        while rgbl.sum() == 3:
            rgbl = np.random.randint(0, 2, 3)
        return "#%02x%02x%02x" % tuple(rgbl*255)

    def on_check_robot_display(self, var):
        """
        Command associated with the checkboxes indicating if a robot must be
        displayed or not.
        :param var: tKinter IntVar, a boolean that is 0 if robot must be hiden,
        1 if robot must be displayed.
        """
        if var.get() == 1:
            var.set(0)
        else:
            var.set(1)

    def _create_circle(self, x, y, r, **kwargs):
        return self.canvas.create_oval(x-r, y-r, x+r, y+r, **kwargs)

    def _create_circle_arc(self, x, y, r, **kwargs):
        if "start" in kwargs and "end" in kwargs:
            kwargs["extent"] = kwargs["end"] - kwargs["start"]
            del kwargs["end"]
        return self.create_arc(x-r, y-r, x+r, y+r, **kwargs)
