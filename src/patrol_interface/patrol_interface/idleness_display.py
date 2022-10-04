from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import tkinter as tk


class IdlenessDisplay(tk.Canvas):
    def __init__(self, master, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.x = [1]
        self.mean_data = [0]
        self.max_data = [0]

        self.fig = Figure(figsize=(6, 6))
        self.a = self.fig.add_subplot(111)
        self.a.set_xlim(right=3600)
        self.a.set_ylim(top=3600)
        self.figure_canvas = FigureCanvasTkAgg(self.fig, master=master)
        self.figure_canvas.get_tk_widget().place(anchor="nw",
                                                 relheight="0.6",
                                                 relwidth="0.4",
                                                 relx="0.55",
                                                 rely="0.05",
                                                 x="0",
                                                 y="0",)
        self.figure_canvas.draw()

    def plot(self):
        self.a.plot(self.x, self.mean_data, color='blue')
        self.a.plot(self.x, self.max_data, color='red')
        self.a.set_title("Mean (blue) and max (red) idleness", fontsize=16)
        self.a.set_ylabel("Idleness", fontsize=14)
        self.a.set_xlabel("Time", fontsize=14)
        self.figure_canvas.draw()

    def update_data_mean_and_max(self, new_time, new_mean, new_max):
        self.x.append(new_time)
        self.mean_data.append(new_mean)
        self.max_data.append(new_max)
        self.plot()
