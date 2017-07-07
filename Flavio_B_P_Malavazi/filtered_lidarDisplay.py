from global_variables import LiDAR
from global_variables import LOCK
import global_variables as VG
import tkinter as ui
import math

class filteredLiDARDisplay(ui.Tk):

    def __init__(self, *args, **kwargs):
        ui.Tk.__init__(self, *args, **kwargs)
        self.title("Filtered LiDAR")
        self._lidar_drawing = ui.Canvas(self, background='white')
        self._lidar_drawing.pack(fill=ui.BOTH, expand=ui.YES, side=ui.TOP)
        self._lidar_drawing.bind('<Configure>', self.resize_event)  # to handle the resizing
        self.geometry("400x400+500+100")
        self._width = 0
        self._height = 0
        self._sizeX = 5000
        self._sizeY = 5000

    def resize_event(self, event):
        self._width = event.width
        self._height = event.height
        self.drawLiFilt()

    def drawLiFilt(self):
        self._lidar_drawing.delete("all")
        self.drawlidarfiltered()

    def drawlidarfiltered(self):
        xrobot = self._sizeX/2
        yrobot = 1000

        LOCK.acquire()
        for data in range(len(VG.data_x_tmp)):
            couleur = "red"

            xmsr = -VG.data_y_tmp[data]+xrobot
            ymsr = VG.data_x_tmp[data]+yrobot

            self._lidar_drawing.create_line(self.real2draw_x(xrobot), self.real2draw_y(yrobot),
                                                self.real2draw_x(xmsr), self.real2draw_y(ymsr), fill=couleur)

            self._lidar_drawing.create_rectangle(self.real2draw_x(xmsr), self.real2draw_y(ymsr),
                                                     self.real2draw_x(xmsr), self.real2draw_y(ymsr),
                                                     fill="red", width=5)

        self._lidar_drawing.create_rectangle(self.real2draw_x(xrobot), self.real2draw_y(yrobot),
                                                 self.real2draw_x(xrobot), self.real2draw_y(yrobot), outline="yellow",
                                                 width=10)
        LOCK.release()

        self.after(150, self.drawLiFilt)

    # to convert an X value from the world to the window frame
    def real2draw_x(self, x):
        step = self._width/float(self._sizeX)
        xdraw = x*float(step)
        return xdraw

    # to convert an Y value from the world to the window frame
    def real2draw_y(self, y):
        step = self._height/float(self._sizeY)
        ydraw = y*float(step)
        return self._height-ydraw
