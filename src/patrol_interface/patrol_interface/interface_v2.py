#!/usr/bin/python3
import tkinter as tk
import tkinter.ttk as ttk
from patrol_interface.mission_display_v2 import MissionDisplayV2
from patrol_interface.idleness_display import IdlenessDisplay


class PatrolInterfaceV2:
    def __init__(self, robot_data, master=None):
        self.display_robot_frames = {}
        self.display_robot_labelframes = {}
        self.display_robot_checkbuttons = {}
        self.display_robot_checkbutton_vars = {}
        self.display_robot_range_checkbuttons = {}
        self.display_robot_range_checkbutton_vars = {}
        self.live_data_robot_frames = {}
        self.live_data_robot_labelframes = {}
        self.live_data_path_labels = {}
        self.live_data_path_values = {}
        self.live_data_next_target_labels = {}
        self.live_data_next_target_values = {}
        self.live_data_robot_checkbutton_vars = {}
        self.live_data_robot_range_checkbuttons = {}
        self.live_data_robot_range_checkbutton_vars = {}
        # build ui
        self.main_frame = ttk.Frame(master)
        self.mission_display = MissionDisplayV2(self.main_frame)
        self.mission_display.configure(cursor="arrow")
        self.mission_display.place(
            anchor="nw",
            relheight="0.6",
            relwidth="0.4",
            relx="0.05",
            rely="0.05",
            x="0",
            y="0",
        )
        self.mission_display.update_display()
        self.idleness_display = IdlenessDisplay(self.main_frame)
        self.option_notebook = ttk.Notebook(self.main_frame)
        self.display_options_frame = ttk.Frame(self.option_notebook)
        self.display_data_frame = ttk.Frame(self.option_notebook)
        self.option_notebook.add(
            self.display_options_frame, text="Display Options")
        self.option_notebook.add(self.display_data_frame, text="Live Data")
        for ind, robot in enumerate(robot_data.keys()):
            self._create_display_options_elements(robot, ind)
            self._create_live_data_options_elements(robot, ind)
        self.option_notebook.configure(height="200", width="200")
        self.option_notebook.place(
            anchor="nw",
            relheight="0.275",
            relwidth="0.9",
            relx="0.05",
            rely="0.7",
            x="0",
            y="0",
        )
        self.main_frame.configure(height="800", width="1200")
        self.main_frame.pack(expand="true", fill="both", side="top")

        # Main widget
        self.mainwindow = self.main_frame

    def _create_display_options_elements(self, robot, ind):
        self.display_robot_frames[robot] = ttk.Frame(
            self.display_options_frame)
        self.display_robot_labelframes[robot] = ttk.Labelframe(
            self.display_robot_frames[robot])
        self.display_robot_checkbuttons[robot] = ttk.Checkbutton(
            self.display_robot_labelframes[robot])
        self.display_robot_checkbutton_vars[robot] = tk.BooleanVar(
            value="1")
        self.display_robot_checkbuttons[robot].configure(
            offvalue="0",
            onvalue="1",
            text="Display Body",
            variable=self.display_robot_checkbutton_vars[robot],
        )
        self.display_robot_checkbuttons[robot].grid(
            column="0", padx="5", pady="5", row=f"{ind}")
        _wcmd = lambda wid=f"robot_{ind}": self.change_display_robot(wid)
        self.display_robot_checkbuttons[robot].configure(command=_wcmd)
        self.display_robot_range_checkbuttons[robot] = ttk.Checkbutton(
            self.display_robot_labelframes[robot])
        self.display_robot_range_checkbutton_vars[robot] = tk.BooleanVar(
            value="1")
        self.display_robot_range_checkbuttons[robot].configure(
            offvalue="0",
            onvalue="1",
            text="Display Com. Range",
            variable=self.display_robot_range_checkbutton_vars[robot],
        )
        self.display_robot_range_checkbuttons[robot].grid(
            column="1", padx="5", pady="5", row=f"{ind}")
        _wcmd = lambda wid=f"robot_{ind}": self.change_display_com_range(
            wid)
        self.display_robot_range_checkbuttons[robot].configure(
            command=_wcmd)
        self.display_robot_labelframes[robot].configure(
            height="200", text=f"{robot}", width="200")
        self.display_robot_labelframes[robot].grid(
            column="0", padx="5", pady="5", row=f"{ind}")
        self.display_robot_frames[robot].configure(
            height="200", width="200")
        self.display_robot_frames[robot].grid(
            column=f"{int(ind/3)}", row=f"{ind%3}")

    def _create_live_data_options_elements(self, robot, ind):
        self.live_data_robot_frames[robot] = ttk.Frame(
            self.display_data_frame)
        self.live_data_robot_labelframes[robot] = ttk.Labelframe(
            self.live_data_robot_frames[robot])
        self.live_data_path_labels[robot] = ttk.Label(
            self.live_data_robot_labelframes[robot])
        self.live_data_path_labels[robot].configure(text="Path: ")
        self.live_data_path_labels[robot].grid(
            column="0", padx="5", pady="5", row="0")
        self.live_data_path_values[robot] = ttk.Label(
            self.live_data_robot_labelframes[robot])
        self.live_data_path_values[robot].configure(anchor="n", text="[ ]")
        self.live_data_path_values[robot].grid(
            column="1", padx="5", pady="5", row="0")
        self.live_data_next_target_labels[robot] = ttk.Label(
            self.live_data_robot_labelframes[robot])
        self.live_data_next_target_labels[robot].configure(
            text="Current Target: ")
        self.live_data_next_target_labels[robot].grid(
            column="2", padx="5", pady="5", row="0"
        )
        self.live_data_next_target_values[robot] = ttk.Label(
            self.live_data_robot_labelframes[robot])
        self.live_data_next_target_values[robot].configure(text=".")
        self.live_data_next_target_values[robot].grid(
            column="3", padx="5", pady="5", row="0")
        self.live_data_robot_labelframes[robot].configure(
            height="200", text=f"{robot}", width="200"
        )
        self.live_data_robot_labelframes[robot].grid(
            column="0", padx="5", pady="5", row="0")
        self.live_data_robot_frames[robot].configure(
            height="200", width="200")
        self.live_data_robot_frames[robot].grid(
            column=f"{int(ind/3)}", row=f"{ind%3}")

    def change_display_robot(self, widget_id):
        pass

    def change_display_com_range(self, widget_id):
        pass


if __name__ == "__main__":
    root = tk.Tk()
    app = PatrolInterfaceV2(root)
    app.run()
