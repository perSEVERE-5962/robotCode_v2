import tkinter as tk
from tkinter import ttk
from networktables import NetworkTables


ROBOT_IP = "10.59.62.2"  # team IP
BG_COLOR = "#000000"     # Black background
FG_COLOR = "#FFFF00"     # Yellow

# Initialize NetworkTables
NetworkTables.initialize(server=ROBOT_IP)
table = NetworkTables.getTable("RobotData")

class CustomUI:
    def __init__(self, root):
        self.root = root
        self.root.title("5962 Robot Dashboard")
        self.root.geometry("900x600")
        self.root.configure(bg=BG_COLOR)

        style = ttk.Style()
        style.theme_use('clam')
        style.configure("TLabel", background=BG_COLOR, foreground=FG_COLOR, font=("Arial", 12))
        style.configure("TButton", background=FG_COLOR, foreground=BG_COLOR, font=("Arial", 12))
        style.map("TButton", background=[("active", "#cccc00")])

        # Header
        header = ttk.Label(self.root, text="FRC TEAM 5962 DASHBOARD", font=("Arial", 18, "bold"))
        header.pack(pady=20)

        # Frame for Auto Selector
        auto_frame = ttk.Frame(self.root)
        auto_frame.pack(side=tk.LEFT, padx=20, pady=10)

        self.canvas = tk.Canvas(auto_frame, bg=BG_COLOR, width=350, height=350, highlightthickness=0)
        self.canvas.pack()

        # Load images
        self.blueBackside = tk.PhotoImage(file=r"C:\FRC\robotCode_v2\src\main\images\blueBackside.png")
        self.noAuto = tk.PhotoImage(file=r"C:\FRC\robotCode_v2\src\main\images\noAuto.png")
        self.redBackside = tk.PhotoImage(file=r"C:\FRC\robotCode_v2\src\main\images\redBackside.png")
        self.blueLeave = tk.PhotoImage(file=r"C:\FRC\robotCode_v2\src\main\images\blueLeave.png")
        self.redLeave = tk.PhotoImage(file=r"C:\FRC\robotCode_v2\src\main\images\redLeave.png")

        self.image = self.canvas.create_image(0, 20, anchor=tk.NW, image=self.noAuto)

        self.output_label = ttk.Label(auto_frame, text="Selected Auto: None")
        self.output_label.pack(pady=10)

        # Buttons for auto selection
        button_frame = ttk.Frame(auto_frame)
        button_frame.pack(pady=10)

        ttk.Button(button_frame, text="No Auto", command=lambda: self.set_auto("NoAuto", self.noAuto)).grid(row=0, column=0, padx=5)
        ttk.Button(button_frame, text="Blue Backside", command=lambda: self.set_auto("BlueBackside", self.blueBackside)).grid(row=0, column=1, padx=5)
        ttk.Button(button_frame, text="Red Backside", command=lambda: self.set_auto("RedBackside", self.redBackside)).grid(row=0, column=2, padx=5)
        ttk.Button(button_frame, text="Blue Leave", command=lambda: self.set_auto("BlueLeave", self.blueLeave)).grid(row=0, column=3, padx=5)
        ttk.Button(button_frame, text="Red Leave", command=lambda: self.set_auto("RedLeave", self.redLeave)).grid(row=0, column=4, padx=5)

        #Frame for Robot Data 
        data_frame = ttk.Frame(self.root)
        data_frame.pack(side=tk.RIGHT, padx=20, pady=10)

        self.speed_label = tk.Label(data_frame, text="Speed: 0.0", fg="yellow", bg="black", font=("Arial", 16))
        self.speed_label.pack(pady=10)

        # Motor Power Input
        entry_label = tk.Label(data_frame, text="Set Motor Power:", fg="yellow", bg="black", font=("Arial", 14))
        entry_label.pack(pady=5)

        self.power_entry = tk.Entry(data_frame)
        self.power_entry.pack(pady=5)

        send_button = tk.Button(data_frame, text="Send", command=self.send_power, bg="yellow", fg="black")
        send_button.pack(pady=10)

        self.alliance_label = tk.Label(data_frame, text="Alliance: UNKNOWN", fg="white", bg="black", font=("Arial", 18, "bold"))
        self.alliance_label.pack(pady=20, fill="x")
        
        # Alliance selection input
        alliance_input_label = tk.Label(data_frame, text="Set Alliance:", fg="yellow", bg="black", font=("Arial", 14))
        alliance_input_label.pack(pady=5)

        # Dropdown for alliance choice
        self.alliance_choice = ttk.Combobox(data_frame, values=["Red", "Blue", "Unknown"])
        self.alliance_choice.current(2)  # Default to "Unknown"
        self.alliance_choice.pack(pady=5)

        # Button to send alliance color to NetworkTables
        set_alliance_button = tk.Button(data_frame, text="Set Alliance", command=self.set_alliance, bg="yellow", fg="black")
        set_alliance_button.pack(pady=5)

        # Start updating data
        self.update_values()


    def set_auto(self, auto_name, img):
        self.canvas.itemconfig(self.image, image=img)
        self.output_label.config(text=f"Selected Auto: {auto_name}")
        table.putString("selectedAuto", auto_name)  # Send selection to robot

    def send_power(self):
        try:
            value = float(self.power_entry.get())
            table.putNumber("motorPower", value)
        except ValueError:
            pass

    def set_alliance(self):
        user_alliance = self.alliance_choice.get()
        table.putString("allianceColor", user_alliance)

    def update_values(self):
        speed = table.getNumber("speed", 0.0)
        self.speed_label.config(text=f"Speed: {speed:.2f}")
        alliance = table.getString("allianceColor", "UNKNOWN").upper()
        if alliance == "RED":
            self.alliance_label.config(text="Alliance: RED", bg="red", fg="white")
        elif alliance == "BLUE":
            self.alliance_label.config(text="Alliance: BLUE", bg="blue", fg="white")
        else:
            self.alliance_label.config(text="Alliance: UNKNOWN", bg="black", fg="white")

        self.root.after(100, self.update_values)


if __name__ == "__main__":
    root = tk.Tk()
    app = CustomUI(root)
    root.mainloop()