import tkinter as tk
from networktables import NetworkTables

class userInterface:
   def __init__(self):
      NetworkTables.initialize(server='10.59.62.2')
      table = NetworkTables.getTable("Auto Chooser")
      #self.someNumberEntry = table.getEntry('idk')
      self.image_1 = tk.PhotoImage(file=r"C:\FRC\robotCode_v2\src\main\images\Red2025.png")
      self.image_2 = tk.PhotoImage(file=r"C:\FRC\robotCode_v2\src\main\images\Blue2025.png")
      self.currentImage = self.image_1
      self.image = self.canvas.create_image(0, 20, anchor=tk.NW, image=self.image_1)
      self.root = tk.Tk()
      self.root.resizable(False, False)

      self.canvas = tk.Canvas(self.root, width = 600, height = 550)
      self.canvas.pack()

      self.B = tk.Button(self.root, text ="Switch Color", command = self.switch_image)
      self.B.place(x=0,y=0)

   def switch_image(self):
    if self.currentImage == self.image_1:
        print("IN IF")
        self.canvas.itemconfig(self.image, image=self.image_2)
        self.currentImage = self.image_2

UI = userInterface()
UI.root.mainloop()