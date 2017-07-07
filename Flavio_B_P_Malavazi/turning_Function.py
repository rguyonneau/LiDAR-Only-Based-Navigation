from global_variables import LiDAR
from global_variables import LOCK
from threading import Thread
from threading import Event as thEvent
import global_variables as VG
import math as math
import time

# This thread is used as a control flow for the automatic mode, when changing rows. The turning
# function manages the flags used by the automatic mode, changing them successively as the robot does the turning.

class turning_Function(Thread):
    def __init__(self):
        Thread.__init__(self)
        self._stop = thEvent()

    def run(self):
        while 1:
            time.sleep(0.08)
            # 1 - Leaving the Row
            if VG.repositionOnRow:
                print("\n##################### Leaving the Row  ########################")
                print("Wait until we get at 1.65m from the end of the row")
                while VG.posx < (1.65 + VG.x_endRow):
                    print("Left Row, DATA:\nIMU = ", VG.IMUStatus, "Theta2 = ", float(VG.theta), "(posx, posy) = (", VG.posx, ",", VG.posy, ")")
                #time.sleep(3)
                LOCK.acquire()
                VG.repositionOnRow = False
                #VG.ParcoursRangee = False
                VG.flagWarning = True
                LOCK.release()

            # 2 - Turning back and searching for the correct position while doing so, the turning back process is
            # divided in two parts, one in which the robot turns back fast, and stops when it reaches the angle of 50º
            # from the rows, and the other, which has OZ going slower in until it finds a model that is perpendicular
            # to him, on the second step, a safety mechanism was added for it not to go over 96 degrees from the rows,
            # in case it loses the optimal position due to data coming from unexpected obstacles
            if VG.turningRow:
                #print("Turning...")

                if VG.flagWarning:
                    #print("\nGoing to sleep...")
                    #time.sleep(3.5)
                    if VG.directionTurn == "Left":
                        while VG.theta < 50:
                            print("Slowing down the turn, DATA: IMU = ", VG.IMUStatus, "Theta2 = ", float(VG.theta), "(posx, posy) = (", VG.posx,",", VG.posy, ")")
                    else:
                        while VG.theta > -50:
                            print("Slowing down the turn, DATA: IMU = ", VG.IMUStatus, "Theta2 = ", float(VG.theta), "(posx, posy) = (", VG.posx,",", VG.posy, ")")

                    print("\nLooking for perpendicular rows...")
                    VG.flagWarning = False
                    #-#print("abs(VG.masterACoeff) = ",abs(VG.masterACoefficient))

                #print("VG.trueRightHandModelPearl2 = ",VG.trueRightHandModelPearl2,"\nVG.currentCRModel = ",VG.currentCRModel)
                LOCK.acquire()
                if (VG.trueRightHandModelPearl2 and (abs(VG.masterACoefficient) < 0.04)) and (abs(VG.theta) > 75) and not VG.flagWarning:
                    print("Rows found with condition 1")
                    print("Data for satisfaction:",VG.trueRightModelDataPearl2)
                    print("Model for Satisfaction:",VG.trueRightHandModelPearl2)
                    VG.turningRow = False
                # Waiting for tests...
                elif ((VG.currentCRModel and ((VG.masterACoefficient != float('inf')) and (VG.masterACoefficient != float('-inf')))) and ((VG.masterACoefficient > 0 and (-0.06 < VG.currentCRModel[0] < 0)) or (VG.masterACoefficient < 0 and (0 > VG.currentCRModel[0] > -0.06)))) and (abs(VG.theta) > 84) and not VG.flagWarning:
                    print("Rows found with condition 2")
                    print("VG.currentCRModel Before satisfaction:",VG.currentCRModel)
                    print("VG.masterACoefficient = ",VG.masterACoefficient)
                    VG.currentCRModel = VG.trueRightHandModelPearl2
                    VG.currentCRModelData = VG.trueRightModelDataPearl2
                    print("Data for satisfaction:", VG.trueRightModelDataPearl2)
                    print("Models for satisfaction:",VG.currentCRModel)
                    #VG.turningRow = False
                elif (abs(VG.theta) > 96):
                    print("Rows found with condition 3")
                    print("VG.currentCRModel Before satisfaction:",VG.currentCRModel)
                    print("VG.masterACoefficient = ",VG.masterACoefficient)
                    VG.currentCRModel = []
                    VG.currentCRModelData = []
                    print("Data for satisfaction:", VG.trueRightModelDataPearl2)
                    print("Models for satisfaction:",VG.currentCRModel)
                    VG.turningRow = False
                LOCK.release()

            # 3 - Searching for the next row while going forward
            if VG.searchingRow:
                #print("Searching for the correct position...")
                if (VG.trueRightHandModelPearl2 and (VG.minimalBCoefficient < 300)):# or (VG.trueLeftHandModelPearl2 and (VG.trueLeftHandModelPearl2[1] > -400)):
                    print("Found correct position...")
                    print("VG.trueRightHandModelPearl2 = ",VG.trueRightHandModelPearl2,"\nVG.currentCRModel = ",VG.currentCRModel)
                    print("Model Data for satisfaction:",VG.trueRightModelDataPearl2)
                    print("Finished adjustements")
                    VG.x_nextRow = VG.posx
                    VG.y_nextRow = VG.posy
                    print("Found the correct position: (VG.x_nextRow,VG.y_nextRow) = (",VG.x_nextRow,",",VG.y_nextRow,")")
                    VG.trueRightHandModelPearl2 = []
                    VG.trueLeftHandModelPearl2 = []
                    VG.trueLeftModelDataPearl2 = []
                    VG.trueRightModelDataPearl2 = []
                    VG.searchingRow = False

            # 4 - Entering the new row (made based on distance to the current row, but may be changed to find the next
            # this would require changing the used filter for finding the next row, may be done but there would have to
            # be room for multiple rows in order to adjust the program)
            if VG.enteringNewRow:
                print("Entering new row...")
                #time.sleep(3)
                if VG.directionTurn == "Left":
                    while VG.posy < VG.y_nextRow + 0.35:
                        print("Almost aligned with the new row, DATA: IMU = ", VG.IMUStatus, "Theta2 = ", float(VG.theta), "(posx, posy) = (", VG.posx, ",",VG.posy, ")")
                else:
                    while VG.posy > VG.y_nextRow - 0.35:
                        print("Almost aligned with the new row, DATA: IMU = ", VG.IMUStatus, "Theta2 = ", float(VG.theta), "(posx, posy) = (", VG.posx, ",",VG.posy, ")")
                VG.x_nextRow = VG.posx
                VG.y_nextRow = VG.posy
                VG.enteringNewRow = False

            # 5 - Reaching the middle of the row (same as 4)
            if VG.reachedMiddleRow:
                print("Reaching the middle of the row")
                #time.sleep(5)
                if VG.directionTurn == "Left":
                    print("VG.averageDist = ",VG.averageDist)
                    if (VG.averageDist < 1500):
                        while VG.posy < VG.y_nextRow + 1.0:
                            print("Reached the middle of the next row, DATA: IMU = ", VG.IMUStatus, "Theta2 = ", float(VG.theta), "(posx, posy) = (", VG.posx, ",",VG.posy, ")")
                        print("Passed Condition while 2")
                        print("VG.y_nextRow = ",VG.y_nextRow)
                        VG.reachedMiddleRow = False
                    else:
                        VG.reachedMiddleRow = False
                else:
                    if (VG.averageDist < 1500):
                        while VG.posy > VG.y_nextRow - 1.0:
                            print("Reached the middle of the next row, DATA: IMU = ", VG.IMUStatus, "Theta2 = ", float(VG.theta), "(posx, posy) = (", VG.posx, ",",VG.posy, ")")
                        VG.reachedMiddleRow = False
                    else:
                        VG.reachedMiddleRow = False

            # 6 - Falling back to face the crop again...
            # Based on the readings from the IMU, goes until theta is within 0,75% of 180º
            if VG.fallBack:
                print("\n######################  Falling back to find the new rows  ######################")
                print("Before:")
                print("IMU = ", VG.IMUStatus, "Theta2 = ", float(VG.theta), "(posx, posy) = (", VG.posx, ",",VG.posy, ")")
                if not VG.repositionComplete:
                    while (((abs(VG.theta) - 180) / 180) < 0.0075) or (((abs(VG.theta) - 180) / 180) < -0.0075):
                        print("IMU = ", VG.IMUStatus, "Theta2 = ", float(VG.theta), "(posx, posy) = (", VG.posx, ",",VG.posy, ")")
                    #time.sleep(6.5)
                    print("After:")
                    print("IMU = ", VG.IMUStatus, "Theta2 = ", float(VG.theta), "(posx, posy) = (", VG.posx, ",",VG.posy, ")")
                    print("Repositioning complete...")
                    VG.waitingReposition = False
                    #VG.finishedTurn = True
                    VG.fallBack = False
                    VG.repositionComplete = True

            # 7 - Moving foward slowly until it finds the new rows that it is about to enter
            if VG.repositionComplete:
                if not VG.waitingReposition:
                    #time.sleep(3)
                    VG.waitingReposition = True
                    VG.finishedTurn = True

               # print("Waiting for reposition")
                if VG.trueRightHandModel and VG.trueLeftHandModel and VG.waitingReposition:
                    print("Succesfully repositioned with both models")
                    flagFirstPassage = True
                    VG.repositionComplete = False
                if VG.waitingReposition and VG.trueRightHandModel and not VG.trueLeftHandModel:
                    print("Repositioned with only right model available")
                    flagFirstPassage = True
                    VG.repositionComplete = False
                if VG.waitingReposition and VG.trueLeftHandModel and not VG.trueRightHandModel:
                    print("Repositioned with only Left model available")
                    flagFirstPassage = True
                    VG.repositionComplete = False

            # 8 - Extra safety step that was deprecated after the usage of the IMU data for the fallBack started being
            # used.
            if VG.stablePosition:

                if flagFirstPassage:
                    print("Waiting for stability...")
                    VG.speedMax = VG.speedMax / 2
                    flagFirstPassage = False
                    #time.sleep(2)

                else:
                    print("Stability Reached!")
                    VG.speedMax = VG.speedMax * 2
                    VG.stablePosition = False

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()


