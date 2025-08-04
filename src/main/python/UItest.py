import tkinter as tk
from tkinter import ttk

BG_COLOR = "#000000"   # Black background
FG_COLOR = "#FFFF00"   # Yellow

class CustomUI:
    def __init__(self, root):
        self.root = root
        self.root.title("5962 UI")
        self.root.geometry("600x400")
        self.root.configure(bg=BG_COLOR)

        style = ttk.Style()
        style.theme_use('clam')
        style.configure("TLabel", background=BG_COLOR, foreground=FG_COLOR, font=("Arial", 12))
        style.configure("TButton", background=FG_COLOR, foreground=BG_COLOR, font=("Arial", 12))
        style.map("TButton", background=[("active", "#cccc00")])

        header = ttk.Label(self.root, text="FRC TEAM 5962 AUTONOMOUS SELECTER", font=("Arial", 18, "bold"))
        header.pack(pady=20)

        self.canvas = tk.Canvas(self.root, bg=BG_COLOR, width=350, height=350, highlightthickness=0)
        self.canvas.pack()

        self.blueBackside = tk.PhotoImage(file=r"C:\FRC\robotCode_v2\src\main\images\blueBackside.png")
        self.noAuto = tk.PhotoImage(file=r"C:\FRC\robotCode_v2\src\main\images\noAuto.png")
        self.redBackside = tk.PhotoImage(file=r"C:\FRC\robotCode_v2\src\main\images\redBackside.png")
        self.blueLeave = tk.PhotoImage(file=r"C:\FRC\robotCode_v2\src\main\images\blueLeave.png")
        self.redLeave = tk.PhotoImage(file=r"C:\FRC\robotCode_v2\src\main\images\redLeave.png")
        self.image = self.canvas.create_image(0,20, anchor=tk.NW, image=self.noAuto)
        self.output_label = ttk.Label(self.root, text="Output will appear here.")
        self.output_label.pack(pady=10)
        # self.entry_label = ttk.Label(self.root, text="Enter your variable here:")
        # self.entry_label.pack()

        # self.entry = ttk.Entry(self.root, font=("Arial", 12))
        # self.entry.pack(pady=5)

        button_frame = ttk.Frame(self.root)
        button_frame.pack(pady=10)

        ttk.Button(button_frame, text="No Auto", command=lambda: self.set_image(self.noAuto)).grid(row=0, column=0, padx=5)
        ttk.Button(button_frame, text="Blue Backside", command=lambda: self.set_image(self.blueBackside)).grid(row=0, column=1, padx=5)
        ttk.Button(button_frame, text="Red Backside", command=lambda: self.set_image(self.redBackside)).grid(row=0, column=2, padx=5)
        ttk.Button(button_frame, text="Blue Leave", command=lambda: self.set_image(self.blueLeave)).grid(row=0, column=3, padx=5)
        ttk.Button(button_frame, text="Red Leave", command=lambda: self.set_image(self.redLeave)).grid(row=0, column=4, padx=5)
        # Put stuff like this here:
        #Sensor readings
        #Live data display
        #Graphs
        #NetworkTables integration

    def set_image(self, img):
        self.canvas.itemconfig(self.image, image=img)
        self.currentImage = img





if __name__ == "__main__":
    root = tk.Tk()
    app = CustomUI(root)
    root.mainloop()
