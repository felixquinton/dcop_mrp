import matplotlib
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import tkinter as tk
import time



class IdlenessDisplay(tk.Tk):
    def __init__(self, *args, **kwargs):
        # Initializes a window
        window = tk.Tk()
        self.window = window

        self.x = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
        self.p = [16.23697, 17.31653, 17.22094, 17.68631, 17.73641,
                  18.6368, 19.32125, 19.31756, 21.20247, 22.41444]

        self.fig = Figure(figsize=(6, 6))
        self.a = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.window)
        self.canvas.get_tk_widget().pack()
        self.plot()
        for i in range(20):
            time.sleep(1)
            self._update_data({10+i: np.random.rand()})
            self.plot()

    def plot(self):
        self.a.plot(self.x, self.p, color='blue')

        self.a.set_title("Estimation Grid", fontsize=16)
        self.a.set_ylabel("Y", fontsize=14)
        self.a.set_xlabel("X", fontsize=14)
        self.canvas.draw()

    def _update_data(self, new_data):
        print(new_data.keys())
        self.x.append(list(new_data.keys())[0])
        self.p.append(new_data[self.x[-1]])


start = IdlenessDisplay()
