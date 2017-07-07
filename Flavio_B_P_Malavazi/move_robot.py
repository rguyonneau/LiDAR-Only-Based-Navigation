import tkinter as Ui
import socket as socket
import global_variables as VG
import numpy as np
from global_variables import LOCK
from automatic_mode import automaticMode
from lidar_thread import LiDARThread

class MoveRobotWithKeyBoard(Ui.Tk):

    def __init__(self, ip, *args, **kwargs):
        Ui.Tk.__init__(self, *args, **kwargs)
        self.configure(background='white')

        self._speed = 120
        self._ip = ip
        self._port = 5555
        self._id = 0xA8
        self._rightWheel = 0
        self._leftWheel = 0
        self._timeSleep = 25
        self._realSideTurn = VG.directionTurn
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.title("Move Oz with the keyboard")
        self.bind("<KeyPress-Up>", self.on_up_press)
        self.bind("<KeyPress-Down>", self.on_down_press)
        self.bind("<KeyPress-Left>", self.on_left_press)
        self.bind("<KeyPress-Right>", self.on_right_press)
        self.bind("<FocusOut>", self.on_focus_out)
        self.bind("<FocusIn>", self.on_focus_in)
        self.bind("<space>", self.on_space_press)
        self.bind("<l>", self.on_l_press)
        self.bind("<q>", self.on_q_press)
        self.bind("<o>", self.on_o_press)
        self.loop_read()

    def on_up_press(self, event):
        #LOCK.acquire()
        """VG.speedLeftMotor = self._speed
        VG.speedRightMotor = self._speed"""
        #LOCK.release()
        self._rightWheel = self._speed
        self._leftWheel = self._speed
        print("Up Key")

    def on_down_press(self, event):
        #LOCK.acquire()
        VG.speedLeftMotor = -self._speed
        VG.speedRightMotor = -self._speed
        #LOCK.release()
        self._rightWheel = -self._speed
        self._leftWheel = -self._speed
        print("Down Key")

    def on_left_press(self, event):
        #LOCK.acquire()
        VG.speedLeftMotor = -self._speed
        VG.speedRightMotor = self._speed
        #LOCK.release()
        self._rightWheel = self._speed
        self._leftWheel = -self._speed
        print("Left Key")

    def on_right_press(self, event):
        #LOCK.acquire()
        VG.speedLeftMotor = self._speed
        VG.speedRightMotor = -self._speed
        #LOCK.release()
        self._rightWheel = -self._speed
        self._leftWheel = self._speed
        print("Right Key")

    def on_l_press(self,event):
        #LOCK.acquire()
        VG.speedLeftMotor = 0
        VG.speedRightMotor = 0
        #LOCK.release()
        self._rightWheel = 0
        self._leftWheel = 0
        print("L Key")

    def on_q_press(self,event):
        #LOCK.acquire()
        VG.resetIMU = True
        print("Q Key")

    def on_space_press(self, event):
        #LOCK.acquire()
        """VG.speedLeftMotor = 0
        VG.speedRightMotor = 0"""

        self._rightWheel = 0
        self._leftWheel = 0

        if VG.Mode_Automatique:
            VG.Mode_Automatique = False
            self.resetFlags()
            print(VG.Mode_Automatique)
        else:
            #VG.HistoriqueRang = [0, 0, 0]
            #VG.Analyse_count = []
            #VG.HistoriqueErreur = []
            VG.Mode_Automatique = True
            print("The automatic mode is now ",VG.Mode_Automatique)
        #LOCK.release()
        print("Space Key")

    def on_focus_out(self, event):
        self.configure(background='red')

    def on_focus_in(self, event):
        self.configure(background='green')

    def on_o_press(self,event):
        print("Performing complete reset")
        LOCK.acquire()
        VG.Mode_Automatique = False
        VG.ParcoursRangee = False
        VG.DemiTours = [False]
        VG.ChercheRang = [False]

        VG.speedRightMotor = 0
        VG.speedLeftMotor = 0

        # PEARL1 VARIABLES
        VG.flagFirst = True
        VG.flagfff = False
        VG.trueRightHandModel = []
        VG.trueLeftHandModel = []
        VG.trueLeftModelData = []
        VG.trueRightModelData = []
        VG.flagInitModels = True
        VG.counterPermanence = 0
        VG.lineHistory = []
        VG.rightHandAverage = 0
        VG.rightHandSum = 0
        VG.leftHandAverage = 0
        VG.leftHandSum = 0
        VG.rightHandDist = 0
        VG.leftHandDist = 0
        VG.rightHandDistSum = 0
        VG.leftHandDistSum = 0
        VG.distSum = []
        VG.averageDist = 0
        VG.targetDistAvg = 0
        VG.targetDistSum = []
        VG.distToTarget = 0
        VG.distToLeft = []
        VG.distToRight = []
        VG.averageDistToRight = 0
        VG.averageDistToLeft = 0
        VG.HistoriqueFiltre2 = []

        # Changing rows variables:
        VG.repositionOnRow = False
        VG.turningRow = False
        VG.flagUsePearl2 = False
        VG.finishedTurn = False
        VG.searchingRow = False
        VG.flagWarning = False
        VG.reachedMiddleRow = False
        VG.enteringNewRow = False
        VG.fallBack = False
        VG.repositionComplete = False
        VG.flagLastRow = False
        VG.stablePosition = False
        VG.directionTurn = self._realSideTurn

        # PEARL 2 Variables:
        VG.counterPermanence2 = 0
        VG.trueRightHandModelPearl2 = []
        VG.trueLeftHandModelPearl2 = []
        VG.trueLeftModelDataPearl2 = []
        VG.trueRightModelDataPearl2 = []

        VG.flagInitModels2 = True
        VG.flagInitModels3 = True
        VG.trueSecondRightData = []
        VG.trueSecondRightModel = []
        VG.trueSecondLeftData = []
        VG.trueSecondLeftModel = []
        VG.minimalBCoefficient = float('inf')
        VG.masterACoefficient = float('inf')

        VG.HistoriqueRang = [0, 0, 0]
        VG.Analyse_count = []
        VG.HistoriqueErreur = []
        VG.HistoriqueFiltre = []
        VG.HistoriqueMagneto = []
        VG.HistoriqueGPS = []

        # Testing box:
        VG.data_x_tmp = []
        VG.data_y_tmp = []
        VG.currentModels = []
        VG.currentData = []
        VG.currentCRModel = []
        VG.currentCRModelData = []
        VG.boxRight = []
        VG.boxLeft = []

        # Inertial Central:
        VG.IMUStatus = 0
        VG.theta = 0
        VG.posx = 0
        VG.posy = 0
        VG.resetIMU = True
        VG.x_endRow = float('inf')
        VG.y_endRow = float('inf')
        VG.x_nextRow = float('inf')
        VG.y_nextRow = float('inf')

        VG.Row_param = []

        VG.HistoriquePtsTransverses = []

        LOCK.release()

    def resetFlags(self):
        LOCK.acquire()
        VG.flagFirst = True
        VG.repositionOnRow = False
        VG.turningRow = False
        VG.flagUsePearl2 = False
        VG.finishedTurn = False
        VG.searchingRow = False
        VG.flagWarning = False
        VG.reachedMiddleRow = False
        VG.enteringNewRow = False
        VG.fallBack = False
        VG.repositionComplete = False
        VG.stablePosition = False

        # Reset flags PEARL2
        VG.counterPermanence2 = 0
        VG.trueRightHandModelPearl2 = []
        VG.trueLeftHandModelPearl2 = []
        VG.trueLeftModelDataPearl2 = []
        VG.trueRightModelDataPearl2 = []
        VG.currentCRModel = []
        VG.currentCRModelData = []

        VG.flagInitModels2 = True
        VG.flagInitModels3 = True
        VG.trueSecondRightData = []
        VG.trueSecondRightModel = []
        VG.trueSecondLeftData = []
        VG.trueSecondLeftModel = []
        VG.minimalBCoefficient = float('inf')
        VG.masterACoefficient = float('inf')

        if (VG.directionTurn == "Left"):
            VG.directionTurn = "Right"
        elif (VG.directionTurn == "Right"):
            VG.directionTurn = "Left"

        automaticMode.Step_Automatique = 0
        LOCK.release()

        return


    def loop_read(self):
        LOCK.acquire()
        if not VG.Mode_Automatique:
            VG.speedRightMotor = self._rightWheel
            VG.speedLeftMotor = self._leftWheel
        """else:
            print("Effective speed for the motors = [",VG.speedLeftMotor,", ",VG.speedRightMotor,"]")"""
        LOCK.release()
        self.after(self._timeSleep, self.loop_read)



