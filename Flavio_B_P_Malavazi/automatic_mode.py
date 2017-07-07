from threading import Thread
from threading import Event as thEvent
import global_variables as VG
from global_variables import LOCK
import time

# Controls the wheels speeds for the robot during the row change procedure, while within a row, it stays put until the
# end of the row is detected, when it assumes the control.

class automaticMode(Thread):
    def __init__(self):
        Thread.__init__(self)
        self._stop = thEvent()
        self._initialTurnD = VG.directionTurn
        self.Step_Automatique = 0


    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def run(self):
        vExec = int((VG.speedTerminal * (VG.speedMax / 100)) / 1)
        while 1:
            if VG.Mode_Automatique:

                # 1 - Initializing the automatic procedure.
                if self.Step_Automatique == 0:
                    VG.ParcoursRangee = True
                    VG.resetIMU = True
                    #print(VG.ParcoursRangee)
                    self.Step_Automatique = 10

                # 2 - The robot follows the controls commands until it founds the end of the row.
                elif self.Step_Automatique == 10:
                    if VG.ParcoursRangee == False:
                        #print("######################  End of the row  ######################")
                        [VG.x_endRow, VG.y_endRow] = [VG.posx,VG.posy]
                        endOfRow = [VG.x_endRow, VG.y_endRow]
                        while VG.repositionOnRow:
                            # Leaving the row...
                            VG.speedRightMotor = int(0.9 * vExec)
                            VG.speedLeftMotor = int(0.9 * vExec)
                            LOCK.acquire()
                            #VG.repositionOnRow = False
                            VG.trueRightHandModel = []
                            VG.trueLeftHandModel = []
                            VG.trueRightModelData = []
                            VG.trueLeftModelData = []
                            LOCK.release()
                        if VG.flagWarning:
                            self.Step_Automatique = 20
                        #self.Step_Automatique = 170

                # 3 - The robot has left the row, it must start turning back.
                elif self.Step_Automatique == 20:
                    if not VG.repositionOnRow:
                        #self.turningFunction()      #VG.DemiTours = [True, "Gauche", 1, VG.Row_param[3]]
                        print("The end of the row was at:",endOfRow)
                        print("Now we're at: IMU = ", VG.IMUStatus, "| Theta2 = ", float(VG.theta), "| (posx, posy) = (", VG.posx, ",", VG.posy,")")
                        print("##################### LEFT THE ROW ########################")
                        #print("######################  Fin demi-tours ICI ######################")

                        """if self.findLastRow():
                            print("Detected last row")
                            self.Step_Automatique = 100
                        else:
                            print("Continuing the rows")"""

                        VG.turningRow = True

                        if VG.directionTurn == "Left":
                            VG.speedRightMotor = -int(0.2 * vExec)
                            VG.speedLeftMotor = -int(0.6 * vExec)
                            #VG.speedRightMotor = -30
                            #VG.speedLeftMotor = -90
                        else:
                            VG.speedRightMotor = -int(0.6 * vExec)
                            VG.speedLeftMotor = -int(0.2 * vExec)
                            #VG.speedRightMotor = -90
                            #VG.speedLeftMotor = -30

                        self.Step_Automatique = 30

                # 4 - The robot is now turning slowly while searching for the perpendicular rows.
                elif self.Step_Automatique == 30:

                    if not VG.flagWarning:
                        #print("Fine Tuning...")

                        if VG.directionTurn == "Left":
                            VG.speedRightMotor = -int(0.06 * vExec)
                            VG.speedLeftMotor = -int(0.30 * vExec)
                            #VG.speedRightMotor = -7
                            #VG.speedLeftMotor = -21
                        else:
                            VG.speedRightMotor = -int(0.30 * vExec)
                            VG.speedLeftMotor = -int(0.06 * vExec)
                            #VG.speedRightMotor = -21
                            #VG.speedLeftMotor = -7

                    if not VG.turningRow:
                        # Found the perpendicular rows, now moving towards the next row.
                        print("\n######################  Going for the new row  ######################")
                        VG.flagUsePearl2 == True
                        VG.searchingRow = True
                        VG.speedRightMotor = int(0.2 * vExec)
                        VG.speedLeftMotor = int(0.2 * vExec)
                        self.Step_Automatique = 40

                # 5 - Still moving to the next row.
                elif self.Step_Automatique == 40:

                    if not VG.searchingRow:
                        #VG.finishedTurn = True
                        VG.enteringNewRow = True
                        VG.speedRightMotor = int(0.2 * vExec)
                        VG.speedLeftMotor = int(0.2 * vExec)
                        #-#print("VG.minimalBCoefficient = ",VG.minimalBCoefficient)
                        VG.reachedMiddleRow = True # ADDED FOR RETURNING FOR THE SAME ROW AS BEFORE
                        self.Step_Automatique = 50

                # 6 - Passing the new row, in order to start the fall back procedure.
                elif self.Step_Automatique == 50:
                    #-#print("2VG.minimalBCoefficient = ", VG.minimalBCoefficient)

                    if not VG.enteringNewRow:
                        VG.speedRightMotor = int(0.3 * vExec)
                        VG.speedLeftMotor = int(0.3 * vExec)
                        VG.reachedMiddleRow = True
                        self.Step_Automatique = 60

                # 7 - Falling back in order to face the crop again.
                elif self.Step_Automatique == 60:
                    if not VG.reachedMiddleRow:
                        #VG.finishedTurn = True
                        VG.fallBack = True

                        if VG.directionTurn == "Left":
                            #VG.speedRightMotor = -27
                            #VG.speedLeftMotor = -81
                            VG.speedRightMotor = -int(0.25 * vExec)
                            VG.speedLeftMotor = -int(0.75 * vExec)
                        else:
                            #VG.speedRightMotor = -81
                            #VG.speedLeftMotor = -27
                            VG.speedRightMotor = -int(0.75 * vExec)
                            VG.speedLeftMotor = -int(0.25 * vExec)

                        self.Step_Automatique = 70

                # 8 -  The new rows are now in front of the robot, advancing slowly while searching for coherent models.
                elif self.Step_Automatique == 70:

                    if not VG.fallBack:
                        VG.speedRightMotor = int(0.3 * vExec)
                        VG.speedLeftMotor = int(0.3 * vExec)
                        self.Step_Automatique = 80

                # 9 - Found coherent models, will wait for stability (gives a small amount of time for the robot to advance slowly towards the new rows).
                elif self.Step_Automatique == 80:

                    if not VG.repositionComplete:
                        VG.stablePosition = True
                        self.Step_Automatique = 90

                # 10 - Stability reached, restarting the procedure.
                elif self.Step_Automatique == 90:

                    if not VG.stablePosition:
                        self.Step_Automatique = 100

                # 11 - MODE AUTOMATIQUE CONCLUDED 1 SUCCESSFULL LOOP.
                elif self.Step_Automatique == 100:
                    print("\nMODE AUTOMATIQUE CONCLUDED 1 SUCCESSFULL LOOP")
                    #&#print("Len of rows so far = ",VG.lenRows,"\nY of rows so far = ",VG.yRows)
                    #VG.Mode_Automatique = False
                    self.resetFlags()
                    VG.Vitesse_moteur_gauche = 0
                    VG.Vitesse_moteur_droit = 0
                    if VG.directionTurn == self._initialTurnD:
                        self.Step_Automatique = 200
                    else:
                        self.Step_Automatique = 0

                # 12 - MODE AUTOMATIQUE CONCLUDED 1 SUCCESSFULL The Crop.
                elif self.Step_Automatique == 200:
                    print("SELF AUTOMATIQUE 200 MODE AUTOMATIQUE CONCLUDED The Crop")
                    # &#print("Len of rows so far = ",VG.lenRows,"\nY of rows so far = ",VG.yRows)
                    # VG.Mode_Automatique = False
                    VG.speedRightMotor = 0
                    VG.speedLeftMotor = 0
                    #self.resetFlags()
                    VG.Mode_Automatique = False
                    self.Step_Automatique = 0

            time.sleep(0.1)

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
        LOCK.release()
        return

    def findLastRow(self):

        print("Entered FindLastRow")

        endRow = False

        if len(VG.lineHistory) > 10:

            if (VG.lineHistory[len(VG.lineHistory) - 1][0] == None) and (VG.lineHistory[len(VG.lineHistory) - 1][2] != None):
                endRow = True

                for i in range(5):
                    if (VG.lineHistory[len(VG.lineHistory) - (i+1)][0] != None):
                        endRow = False

            elif (VG.lineHistory[len(VG.lineHistory) - 1][2] == None):
                endRow = True

                for i in range(5):
                    if (VG.lineHistory[len(VG.lineHistory) - (i+1)][2] != None):
                        endRow = False

        if not endRow:
            print("row in Extreme position")

        if endRow:
            if (VG.directionTurn == "Right"):
                VG.directionTurn = "Left"
                VG.flagLastRow = False
                print("Returning 1")
                return 1
            elif (VG.directionTurn == "Left"):
                VG.directionTurn = "Right"
                VG.flagLastRow = False
                print("Returning 1")
                return 1

        else:
            print("row in Extreme position")
            return 0