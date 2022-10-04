import tkinter as tk
import tkinter.scrolledtext as st


class AuctionDisplay(tk.Tk):
    def __init__(self, *args, **kwargs):
        # Initializes a window
        tk.Tk.__init__(self, *args, **kwargs)

        self.x = [1]
        self.mean_data = [0]
        self.max_data = [0]

        self.geometry("600x400")

        self.text_area = st.ScrolledText(self, width=70, height=19,
                                         font=("Times New Roman", 12))

        self.text_area.grid(column=0, pady=10, padx=10)

    def update_auction_text(self, text):
        self.text_area.insert(tk.INSERT, text + "\n")
