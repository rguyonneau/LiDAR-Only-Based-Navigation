import numpy as np
import math as math
import tkinter as ui
import time as time
import random
import global_variables as VG
from global_variables import LiDAR
from global_variables import LOCK
from minxmaxfunc import *
from filtered_lidarDisplay import filteredLiDARDisplay
#from accel_Display import AccelDisplaylb

class PEARL(ui.Tk):

    def __init__(self, *args, **kwargs):
        ui.Tk.__init__(self, *args, **kwargs)
        self.title("PEARL")
        self._lidar_drawing = ui.Canvas(self, background='white')
        self._lidar_drawing.pack(fill=ui.BOTH, expand=ui.YES, side=ui.TOP)
        self._lidar_drawing.bind('<Configure>', self.resize_event)  # to handle the resizing
        self.Filtree = ui.Canvas(self, background='white')
        self.Filtree.pack(fill=ui.BOTH, expand=ui.YES, side=ui.TOP)
        self.DeltaScreen = ui.Canvas(self, background='yellow')
        self.DeltaScreen.pack(fill=ui.BOTH, expand=ui.YES, side=ui.TOP)
        """self.AccelScreen = ui.Canvas(self, background='yellow')
        self.AccelScreen.pack(fill=ui.BOTH, expand=ui.YES, side=ui.TOP)"""
        self.geometry("400x1000+1000+0")
        self._width = 0
        self._height = 0
        self._sizeX = 6000
        self._sizeY = 6000
        self.initialOp = True
        self._data = []
        self._data_lines = []
        self._initSpeedMax = VG.speedMax
        self._erThresh = 10
        self.find_lines()
    # -----------------------------------------------------------------------------------------------
    def resize_event(self, event):
        self._width = event.width
        self._height = event.height
        # self.draw()
    # -----------------------------------------------------------------------------------------------
    # PEARL Functions:
    # -----------------------------------------------------------------------------------------------
    # Find and propose the initial number of straight line models using 4 points for each model in order to improve
    # the chances of orienting them in the right direction.
    def initialStep2(self, x_data, y_data, labels, nbOfModels):

        models = []
        randomlyChosenXGlobal = []
        randomlyChosenYGlobal = []

        #print("x_data = ",x_data,"\ny_data = ",y_data)

        # Giving all points the initial -1 label, so that they don't belong to any model.
        for k in range(len(labels)):
            labels[k] = -1

        # Choosing 4 random points amongst for each model, out of those who are free:
        for element in range(nbOfModels):

            randomlyChosenX = []
            randomlyChosenY = []

            while True:
                index1 = random.randint(0, (len(x_data) - 1))
                if (labels[index1] < 0):
                    labels[index1] = element + 1
                    #print(x_data[index1])
                    randomlyChosenX.append(float(x_data[index1]))
                    randomlyChosenY.append(float(y_data[index1]))
                    break
            while True:
                index2 = random.randint(0, (len(x_data) - 1))
                if ((labels[index2] < 0) and (x_data[index2] != x_data[index1])):
                    labels[index2] = element + 1
                    randomlyChosenX.append(float(x_data[index2]))
                    randomlyChosenY.append(float(y_data[index2]))
                    break
            while True:
                index3 = random.randint(0, (len(x_data) - 1))
                if ((labels[index3] < 0) and (x_data[index3] != x_data[index2])):
                    labels[index3] = element + 1
                    randomlyChosenX.append(float(x_data[index3]))
                    randomlyChosenY.append(float(y_data[index3]))
                    break
            while True:
                index4 = random.randint(0, (len(x_data) - 1))
                if ((labels[index4] < 0) and (x_data[index4] != x_data[index3])):
                    labels[index4] = element + 1
                    randomlyChosenX.append(float(x_data[index4]))
                    randomlyChosenY.append(float(y_data[index4]))
                    break

            # Proposing an initial simple model for the 4 points based only at the two first ones (this guarantees that
            # the model will go through at least two of the available points)
            dx = float(x_data[index2]) - float(x_data[index1])
            dy = float(y_data[index2]) - float(y_data[index1])
            a = dy / dx
            b = float(y_data[index2]) - (a * float(x_data[index2]))

            model = ([a, b, element + 1, int(4), 0])

            # Going for an initial models reestimation based on minimizing the distance between all points and the
            # model that they are given
            model = self.reEstimationModel(randomlyChosenX, randomlyChosenY, model)

            models.append(model)

        #print("models =\n",models)
        #print("\n<-- Leaving initialStep2")
        # models,labels = modelsKillInit(labels,models,(5*threshParallel),threshRatio)
        #print("We have:",len(models),"models\nAnd",len(x_data),"Data points...")
        return labels, models
    # -----------------------------------------------------------------------------------------------
    # Re-Estimating the models using the points within the labels and minimizing the sum of distances between the
    # rest of the points in the label and it's model.
    def reEstimationModel(self, x_pointsInModel, y_pointsInModel, model):


        #print("\n--> Entering reEstimationModel...")
        distOutlier = float('inf')  # 1200

        previousEnergy = model[4]
        numbOfInliers = model[3]
        analisedModel = model[2]

        # print("Remodelando modelo",model[2],"=",model)

        distAnt = distOutlier  # models[modelCounter-1][4]
        kCoeff_temp = model[0]
        interCoeff_temp = model[1]

        # Testing all possible pairs of points within the model for a new model that minimizes the distances between
        # all points and the model
        for point1 in range(len(x_pointsInModel)):

            for point2 in range(point1 + 1, (len(x_pointsInModel) - 1)):

                dx_mod = x_pointsInModel[point2] - x_pointsInModel[point1]
                dy_mod = y_pointsInModel[point2] - y_pointsInModel[point1]

                if (dx_mod != 0):
                    a_mod = dy_mod / dx_mod
                    b_mod = y_pointsInModel[point2] - (a_mod * x_pointsInModel[point2])

                    # Calculating the total distance between points and model with this 2 points as model centers

                    totalDist = self.calcSomDist(x_pointsInModel, y_pointsInModel, a_mod, b_mod)

                    # If we've achieved a result that has a smaller sum of distance, then that's our new provisional
                    # model for the label
                    if (totalDist < distAnt):
                        distAnt = totalDist
                        kCoeff_temp = a_mod
                        interCoeff_temp = b_mod
                else:
                    # print("I've entered the else...")
                    a_mod = 1000
                    b_mod = y_pointsInModel[point2] - (a_mod * x_pointsInModel[point2])
                    if (totalDist < distAnt):
                        distAnt = totalDist
                        kCoeff_temp = a_mod
                        interCoeff_temp = b_mod

        # After the process is completed, we have a new, optimized model that is returned.
        newModel = ([float(kCoeff_temp), float(interCoeff_temp), analisedModel, int(numbOfInliers), distAnt])
        # print("@@@@ The new model is now: @@@@\n",newModel)
        #print("<-- Leaving reEstimationModel...")
        return newModel
    # ------------------------------------------------------------------------------------------------
    # Amongst the outliers, search for new models
    def searchNewModels2(self, x_data, y_data, labels, models):

        #print("\n--> Entering searchNewModels2...")
        # Find and propose the initial number of straight line models
        outLiers = 0
        randomlyChosenXGlobal = []
        randomlyChosenYGlobal = []
        nbAntMod = len(models)
        # print("models Before searchNewModels =",models)

        for i in range(len(labels)):
            if (labels[i] == 0):
                outLiers = outLiers + 1

        nbOfModels = int(outLiers / 8)

        for element in range(nbOfModels):

            randomlyChosenX = []
            randomlyChosenY = []

            while True:
                index1 = random.randint(0, (len(x_data) - 1))
                if (labels[index1] == 0):
                    labels[index1] = -1
                    randomlyChosenX.append(float(x_data[index1]))
                    randomlyChosenY.append(float(y_data[index1]))
                    break
            while True:
                index2 = random.randint(0, (len(x_data) - 1))
                if ((labels[index2] == 0) and (x_data[index2] != x_data[index1])):
                    labels[index2] = -1
                    randomlyChosenX.append(float(x_data[index2]))
                    randomlyChosenY.append(float(y_data[index2]))
                    break
            while True:
                index3 = random.randint(0, (len(x_data) - 1))
                if ((labels[index3] == 0) and (x_data[index3] != x_data[index2])):
                    labels[index3] = -1
                    randomlyChosenX.append(float(x_data[index3]))
                    randomlyChosenY.append(float(y_data[index3]))
                    break
            while True:
                index4 = random.randint(0, (len(x_data) - 1))
                if ((labels[index4] == 0) and (x_data[index4] != x_data[index3])):
                    labels[index4] = -1
                    randomlyChosenX.append(float(x_data[index4]))
                    randomlyChosenY.append(float(y_data[index4]))
                    break

            dx = float(x_data[index2]) - float(x_data[index1])
            dy = float(y_data[index2]) - float(y_data[index1])
            a = dy / dx
            b = float(y_data[index2]) - (a * float(x_data[index2]))

            if (nbAntMod != 0):
                model = ([a, b, (models[nbAntMod - 1][2] + element + 1), int(4), 0])
                model = self.reEstimationModel(randomlyChosenX, randomlyChosenY, model)
                models.append(model)
                labels[index1] = labels[index2] = labels[index3] = labels[index4] = model[2]
            else:
                model = ([a, b, (element + 1), int(4), 0])
                model = self.reEstimationModel(randomlyChosenX, randomlyChosenY, model)
                models.append(model)
                labels[index1] = labels[index2] = labels[index3] = labels[index4] = model[2]
                # print("models after adding the new one:\n",models)

        # print("models after searchNewModels2=\n",models)
        #print("\n<-- Leaving searchNewModels2")
        return models, labels
    # ------------------------------------------------------------------------------------------------
    # Function to merge the models that are closer than a defined threshold
    def modelsMS2(self, models, labels):

        #print("\n--> Entering modelsMS2...")
        nbAntMod = len(models)
        # print("We have",len(models),"models")
        equalModels = []
        flagEqual = False
        counterModel1 = 0
        counterModel2 = 0

        for element in range(len(models)):
            for i in range(element + 1, len(models)):
                if (i != element):
                    if ((abs(models[element][0] - models[i][0]) < VG.kCoefThresh) and (
                        abs(models[element][1] - models[i][1]) < VG.bCoefThresh)):
                        equalModels.append([int(models[element][2]), int(models[i][2])])
                        # print("Found Equal models!")
                        # print("\nThe equal models are: \n",models[i],"\nAnd\n",models[element])
                        #print(equalModels)
                        flagEqual = True

        if flagEqual:

            for i in range(len(equalModels)):
                modelsLabels = []
                for model in range(len(models)):
                    modelsLabels.append(models[model][2])
                if (len(modelsLabels) > 1):

                    try:
                        index1 = modelsLabels.index(equalModels[i][0])
                    except ValueError:
                        index1 = modelsLabels.index(replacement)
                    try:
                        index2 = modelsLabels.index(equalModels[i][1])
                    except ValueError:
                        index2 = modelsLabels.index(replacement)

                    if (index1 != index2):
                        if ((index1 != -2) and (index2 != -2)):
                            counterModel1 = models[index1][3]
                            counterModel2 = models[index2][3]

                        model1 = models[index1][2]
                        model2 = models[index2][2]

                        if (counterModel2 <= counterModel1):
                            modelToBeDeleted = model2
                            replacement = model1
                        else:
                            modelToBeDeleted = model1
                            replacement = model2

                        coeffK1 = models[index1][0]
                        intercept1 = models[index1][1]

                        coeffK2 = models[index2][0]
                        intercept2 = models[index2][1]

                        newCoeffK = (coeffK1 + coeffK2) / 2
                        newIntercept = (intercept1 + intercept2) / 2

                        if (modelToBeDeleted == model2):
                            models[index1][0] = newCoeffK
                            models[index1][1] = newIntercept
                            models[index1][3] = int(models[index1][3] + models[index2][3])
                            models[index1][4] = models[index1][4] + models[index2][4]
                            for j in range(len(labels)):
                                if (labels[j] == model2):
                                    labels[j] = model1
                        else:
                            models[index2][0] = newCoeffK
                            models[index2][1] = newIntercept
                            models[index2][3] = int(models[index1][3] + models[index2][3])
                            models[index2][4] = models[index1][4] + models[index2][4]
                            for j in range(len(labels)):
                                if (labels[j] == model1):
                                    labels[j] = model2

                        # print("newModel[",posiModel1,"] = ",models[posiModel1])

                        for j in range(len(models)):
                            if (models[j][2] == modelToBeDeleted):
                                del models[j]
                                break

        # if (len(models) == 0):
        # print("######## THERE WAS AN ERROR IN modelsMS2 ########")
        #print("<-- Leaving modelsMS2...")
        # print("We now have",len(models),"models")

        return models, labels
    # ------------------------------------------------------------------------------------------------
    # Alpha expansion of the label l.
    # At this function, we go through all points and models checking if the points are closer to any model than
    # they are to the outliers pool (that has a fixed distance to all points)
    def expansionEnergy(self, x_data, y_data, labels, models):

        newEnergy = 0
        nbOfCurrModels = len(models)
        factor = 1
        # (float('inf'))#10000#

        for model in range(len(models)):
            models[model][4] = 0
            models[model][3] = 0

        for p in range(len(x_data)):

            minDist = VG.distOutlier

            for element in range(len(models)):
                # all the distances between the points and the models can be described as:
                distAt = factor * abs(
                    (float(y_data[p]) - models[element][0] * float(x_data[p]) - models[element][1]) / math.sqrt(
                        math.pow(models[element][0], 2) + 1.0))
                # print(distAt)

                if (distAt < minDist):
                    minDist = distAt
                    labels[p] = models[element][2]
                    dominantModel = element

            if (minDist >= VG.distOutlier):
                labels[p] = 0
            else:
                models[dominantModel][3] = models[dominantModel][3] + 1
                models[dominantModel][4] = models[dominantModel][4] + minDist

            newEnergy = minDist + newEnergy

        for p in range(len(labels)):
            for q in range((p + 1), (len(x_data) - 1)):
                if (labels[p] != labels[q]) and (labels[p] > 0) and (labels[q] > 0):  #:#
                    w_pq = math.exp(-(math.sqrt(math.pow((float(x_data[p]) - float(x_data[q])), 2) + math.pow(
                        (float(y_data[p]) - float(y_data[q])), 2))) / math.pow(VG.delta, 2))
                    newEnergy = VG.alpha * w_pq + newEnergy

        # We proceed to kill the models that have less than 4 points after the expansion, this means that initial
        # models that were proposed and found no other inliers than the points that were already atributed to them are
        # not acceptable and must be rejected.
        newModels, labels = self.modelsShrinkEx(x_data, y_data, models, labels)

        # We penalize the energy based on the number of outliers in order to avoind settling for less models than the
        # ideal (that is unknown)
        countOutliers = 0
        for point in range(len(labels)):
            if (labels[point] == 0):
                countOutliers = countOutliers + 1
        newEnergy = newEnergy + (countOutliers * 1000)

        # if (len(newModels) != len(models)):
        # print("\nI've killed",(len(models)-len(newModels)),"Doing the expansion\n")

        #print("<-- Leaving expansionEnergy...")

        return newModels, labels, newEnergy
    # -----------------------------------------------------------------------------------------------
    # Killing models that got only 4 points in expansion.
    def modelsShrinkEx(self, x_data, y_data, models, labels):

        #print("\n--> Entered modelsShrinkEx...")
        nbAntMod = len(models)
        modelsToBeDeleted = []

        for model in range(len(models)):

            if (models[model][3] <= 4):
                modelsToBeDeleted.append(int(models[model][2]))
                for element in range(len(labels)):
                    if (labels[element] == models[model][2]):
                        labels[element] = 0

        for i in range(len(modelsToBeDeleted)):

            for j in range(len(models)):
                if (models[j][2] == modelsToBeDeleted[i]):
                    del models[j]
                    break

                    # if (len(models) == 0):
                    # print("######## THERE WAS AN ERROR IN modelsShrinkEx ########")
        #print((nbAntMod-len(models)),"Model(s) was(were) distroyed, now we have:",len(models),"models\n<-- Leaving modelsShrinkEx.\n",)
        # findEnergy0(models)
        return models, labels
    # -----------------------------------------------------------------------------------------------
    # Re-Estimating the models using the points within the labels and minimizing the sum of distances between the rest
    # of the points in the label and it's model
    def reEstimation2(self, x_data, y_data, labels, models):
        #print("\n--> Entering reestimation2...")
        modelCounter = (len(models))
        distOutlier = 800  # float('inf')#
        newModels = []
        flagCont = False

        while (modelCounter != 0):
            x_pointsInModel = []
            y_pointsInModel = []
            previousEnergy = models[modelCounter - 1][4]
            numbOfInliers = models[modelCounter - 1][3]
            analisedModel = models[modelCounter - 1][2]

            # print("\n\n\nRemodelando modelo models[modelCounter] =",models[modelCounter-1],"\nModelCounter =",modelCounter)

            for element in range(len(x_data)):
                if (labels[element] == analisedModel):
                    x_pointsInModel.append(float(x_data[element]))
                    y_pointsInModel.append(float(y_data[element]))
                    labels[element] = modelCounter

            distAnt = models[modelCounter - 1][4]
            kCoeff_temp = models[modelCounter - 1][0]
            interCoeff_temp = models[modelCounter - 1][1]

            for point1 in range(len(x_pointsInModel)):

                for point2 in range(point1 + 1, (len(x_pointsInModel) - 1)):
                    dx_mod = x_pointsInModel[point2] - x_pointsInModel[point1]
                    dy_mod = y_pointsInModel[point2] - y_pointsInModel[point1]
                    try:
                        a_mod = dy_mod / dx_mod
                    except ZeroDivisionError:
                        a_mod = kCoeff_temp
                    # print("I'm in model:",analisedModel,"\npoint 1 =",point1,":",x_pointsInModel[point1],";",y_pointsInModel[point1],"\npoint 2 =",point2,":",x_pointsInModel[point2],";",y_pointsInModel[point2],"The models are:\n",models)
                    b_mod = y_pointsInModel[point2] - (a_mod * x_pointsInModel[point2])
                    # if not (point2 % 5):
                    # print("passando pelos pontos\np1:",point1,"\np2:",point2)
                    # print("dx_mod =",dx_mod,"\ndy_mod =",dy_mod,"\na_mod =",a_mod,"\nb_mod",b_mod)

                    totalDist = self.calcSomDist(x_pointsInModel, y_pointsInModel, a_mod, b_mod)
                    # totalDist = calcSomDist2(x_pointsInModel,y_pointsInModel,a_mod,b_mod,point1,point2)

                    if (totalDist < distAnt):
                        # print("### Menor soma total de Distancia encontrada ###\ndistAnt = ",distAnt,"\ntotalDist =",totalDist)
                        # print("Foi com os pontos:\np1 =",point1,"\np2 =",point2)
                        # print("dx_mod =",dx_mod,"\ndy_mod =",dy_mod,"\na_mod =",a_mod,"\nb_mod",b_mod)
                        distAnt = totalDist
                        # print("Total Distancia anterior = ",distAnt)
                        kCoeff_temp = a_mod
                        interCoeff_temp = b_mod

            newModels.append(
                [float(kCoeff_temp), float(interCoeff_temp), modelCounter, int(numbOfInliers), previousEnergy])
            # print("@@@@ The new models are now: @@@@\n",newModels)
            modelCounter = modelCounter - 1

        newModels.sort(key=lambda newModels: newModels[2])
        # findEnergy0(newModels)
        #print("<-- Leaving reestimation2...")
        return newModels, labels
    # -----------------------------------------------------------------------------------------------
    # Calculating distance between point and straight line:
    def calcSomDist(self, x, y, a_mod, b_mod):
        dist = 0
        for element in range(len(x)):
            dist = dist + abs((y[element] - a_mod * x[element] - b_mod) / math.sqrt(math.pow(a_mod, 2) + 1.0))
        return dist
    # -----------------------------------------------------------------------------------------------
    # Checking if the models have a minimum number of points in order to be maintained.
    def modelsShrink(self, x_data, y_data, labels, models,nbMinPoints):

        nbAntMod = len(models)
        modelsToBeDeleted = []

        for model in range(len(models)):
            nbOfElements = models[model][3]

            if (nbOfElements < nbMinPoints):
                modelsToBeDeleted.append(int(models[model][2]))
                for element in range(len(labels)):
                    if (labels[element] == models[model][2]):
                        labels[element] = 0

        for i in range(len(modelsToBeDeleted)):

            for j in range(len(models)):
                if (models[j][2] == modelsToBeDeleted[i]):
                    del models[j]
                    break

                    # if (len(models) == 0):
                    # print("######## THERE WAS AN ERROR IN modelsShrink ########")
        # print("I've distroyed:",(nbAntMod-len(models)),"Model(s)")
        #print("<-- Leaving modelsShrink. ")#,(nbAntMod-len(models)),"Model(s) was(were) distroyed, now we have:",len(models),"models")
        # findEnergy0(models)
        return models, labels
    # -----------------------------------------------------------------------------------------------
    # Checking if the models have a minimum number of points in order to be maintained.
    def modelsShrinkInit(self, x_data, y_data, labels, models,nbMinPoints):
        nbAntMod = len(models)
        modelsToBeDeleted = []

        for model in range(len(models)):
            nbOfElements = models[model][3]

            if (nbOfElements < nbMinPoints):
                modelsToBeDeleted.append(int(models[model][2]))
                for element in range(len(labels)):
                    if (labels[element] == models[model][2]):
                        labels[element] = 0

        for i in range(len(modelsToBeDeleted)):

            for j in range(len(models)):
                if (models[j][2] == modelsToBeDeleted[i]):
                    del models[j]
                    break

                    # if (len(models) == 0):
                    # print("######## THERE WAS AN ERROR IN modelsShrink ########")
        # print("I've distroyed:",(nbAntMod-len(models)),"Model(s)")
        #print("<-- Leaving modelsShrinkInit. ")#,(nbAntMod-len(models)),"Model(s) was(were) distroyed, now we have:",len(models),"models")
        # findEnergy0(models)
        return models, labels
    # -----------------------------------------------------------------------------------------------
    # Destroying models that have a poor coherency to the others (aren't parallel with any of them) or have too few
    # inliers compared to the amount of available data points.
    def modelsKill3(self, labels, models,threshRatio):

        modelsLabels = []
        modelsToBeDeleted = []
        bestRatio = float('inf')
        nbAntModels = len(models)
        # print("\nmodels before energy division:",models)
        countParallels = np.zeros(len(models))

        safeModels = self.findParallels2(models)
        # print("safeModels = ",safeModels)

        # print("safeModels = ",safeModels)

        for model in range(len(models)):
            modelsLabels.append(models[model][2])

        for k in range(len(safeModels)):

            try:
                index = modelsLabels.index(safeModels[k])
            except ValueError:
                index = -2

            if (index != -2):
                models[index][4] = models[index][4] / 5
                countParallels[index] = countParallels[index] + 1

        # print("\nmodels after energy division:",models)

        """for i in range(len(countParallels)):
            if (countParallels[i] == 0):
                models[i][4] = 2*models[i][4]"""

        for model in range(len(models)):
            if (models[model][4] / (models[model][3] * (countParallels[model] + 1)) < bestRatio):
                bestModel = models[model]
                bestRatio = models[model][4] / (models[model][3] * (countParallels[model] + 1))
            if not (models[model][4] / models[model][3] <= threshRatio):
                modelsToBeDeleted.append(int(models[model][2]))

        for i in range(len(modelsToBeDeleted)):

            for j in range(len(models)):
                if (models[j][2] == (modelsToBeDeleted[i])):
                    del models[j]
                    for k in range(len(labels)):
                        if (labels[k] == modelsToBeDeleted[i]) and (labels[k] != bestModel[2]):
                            labels[k] = 0
                    break

        if (len(models) == 0):
            # print("######## THERE WAS AN ERROR IN modelsKill3 ########")
            # print("The best model we had was:",bestModel,"and it has been reappended to models")
            models.append(bestModel)
        # print((nbAntModels - len(models)),"model(s) was(were) destroyed here, now we have:",len(models),"models.\nThe best one is:",bestModel)
        #print("<-- Leaving modelsKill3...")
        return models, labels
    # -----------------------------------------------------------------------------------------------
    # Checking if two models are parallel
    def findParallels2(self, models):

        parallel = []
        safeModels = []

        for model1 in range(len(models)):
            for model2 in range(model1 + 1, (len(models) - 1)):
                if (abs(models[model1][0] - models[model2][0]) <= VG.threshParallel):
                    parallel.append([int(models[model1][2]), int(models[model2][2])])

        for k in range(len(parallel)):
            safeModels.append(parallel[k][0])
            safeModels.append(parallel[k][1])

        """print("\nThe Parallel models are:",parallel)
        print("\nsafeModels is:",safeModels)
        print("All models are: ",models)"""
        return safeModels  # parallel,
    # -----------------------------------------------------------------------------------------------
    # Saving current models and labels for posterior usage for the end of the algorithm
    def saveModelsLabels(self, models, labels):
        # print("\n--> Entering saveModelsLabels...")
        savedModels = []
        savedLabels = []

        for model in range(len(models)):
            savedModels.append(models[model])
        for label in range(len(labels)):
            savedLabels.append(labels[label])
        # print("<-- Leaving saveModelsLabels\n")
        return savedModels, savedLabels
    # -----------------------------------------------------------------------------------------------
    # Finding parallel straight lines amongst the models we already have:
    def findParallels(self, models, threshParallel):
        parallel = []
        safeModels = []
        for model1 in range(len(models)):
            for model2 in range(model1 + 1, (len(models) - 1)):
                if (abs(models[model1][0] - models[model2][0]) <= threshParallel):
                    parallel.append([int(model1 + 1), int(model2 + 1)])

        for k in range(len(parallel)):
            if (parallel[k][0] not in safeModels):
                safeModels.append(parallel[k][0])
            if (parallel[k][1] not in safeModels):
                safeModels.append(parallel[k][1])

        """print("\nThe Parallel models are:",parallel)
        #print("\nsafeModels is:",safeModels)
        #print("All models are: ",models)"""
        return parallel  # ,safeModels#
    # -----------------------------------------------------------------------------------------------
    # END OF PEARL FUNCTIONS
    # -----------------------------------------------------------------------------------------------
    # Functions for drawing (Using Franck's functions for this part):
    # -----------------------------------------------------------------------------------------------
    def draw(self, Clustered_data, Curve):
        self._lidar_drawing.delete("all")
        colors = ["gray13", "red2", "chocolate2", "LightGoldenrod3", "SeaGreen2",
                  "LightSteelBlue2", "cornsilk4", "salmon", "deep sky blue", "navy",
                  "goldenrod", "blue violet", "dark orchid", "LavenderBlush4", "MistyRose2", "LightCyan4",
                  "LemonChiffon3", "LemonChiffon3"]
        c = 0
        for droite in Clustered_data:
            for point in droite:
                self._lidar_drawing.create_rectangle(self.real2draw_x(point[0] * math.cos(math.radians(-90)) +
                                                                      point[1] * math.sin(math.radians(-90))),
                                                     self.real2draw_y(-point[0] * math.sin(math.radians(-90)) +
                                                                      point[1] * math.cos(math.radians(-90))),
                                                     self.real2draw_x(point[0] * math.cos(math.radians(-90)) +
                                                                      point[1] * math.sin(math.radians(-90))),
                                                     self.real2draw_y(-point[0] * math.sin(math.radians(-90)) +
                                                                      point[1] * math.cos(math.radians(-90))),
                                                     fill=colors[c], outline=colors[c], width=2)
            if len(Curve) > c:
                x1 = Curve[c][2]
                y1 = Curve[c][0] * Curve[c][2] + Curve[c][1]
                x2 = Curve[c][3]
                y2 = Curve[c][0] * Curve[c][3] + Curve[c][1]
                self._lidar_drawing.create_line(self.real2draw_x(x1 * math.cos(math.radians(-90)) +
                                                                 y1 * math.sin(math.radians(-90))),
                                                self.real2draw_y(-x1 * math.sin(math.radians(-90)) +
                                                                 y1 * math.cos(math.radians(-90))),
                                                self.real2draw_x(x2 * math.cos(math.radians(-90)) +
                                                                 y2 * math.sin(math.radians(-90))),
                                                self.real2draw_y(-x2 * math.sin(math.radians(-90)) +
                                                                 y2 * math.cos(math.radians(-90))), fill=colors[c])
            self._lidar_drawing.create_rectangle(self.real2draw_x(0), self.real2draw_y(100),
                                                 self.real2draw_x(0), self.real2draw_y(100), outline="yellow",
                                                 width=10)
            c += 1

            # to convert an X value from the world to the window frame
    # -----------------------------------------------------------------------------------------------
    def draw2(self, Clustered_data, Curve):
        try:
            self.Filtree.delete("all")
            colors = ["gray13", "red2", "chocolate2", "LightGoldenrod3", "SeaGreen2",
                      "LightSteelBlue2", "cornsilk4", "salmon", "deep sky blue", "navy",
                      "goldenrod", "blue violet", "dark orchid", "LavenderBlush4", "MistyRose2", "LightCyan4",
                      "LemonChiffon3", "LemonChiffon3"]
            c = 0
            for droite in Clustered_data:
                for point in droite:
                    self.Filtree.create_rectangle(self.real2draw_x(point[0] * math.cos(math.radians(-90)) +
                                                                   point[1] * math.sin(math.radians(-90))),
                                                  self.real2draw_y(-point[0] * math.sin(math.radians(-90)) +
                                                                   point[1] * math.cos(math.radians(-90))),
                                                  self.real2draw_x(point[0] * math.cos(math.radians(-90)) +
                                                                   point[1] * math.sin(math.radians(-90))),
                                                  self.real2draw_y(-point[0] * math.sin(math.radians(-90)) +
                                                                   point[1] * math.cos(math.radians(-90))),
                                                  fill=colors[c], outline=colors[c], width=2)
                if len(Curve) > c:
                    x1 = Curve[c][2]
                    y1 = Curve[c][0] * Curve[c][2] + Curve[c][1]
                    x2 = Curve[c][3]
                    y2 = Curve[c][0] * Curve[c][3] + Curve[c][1]
                    self.Filtree.create_line(self.real2draw_x(x1 * math.cos(math.radians(-90)) +
                                                              y1 * math.sin(math.radians(-90))),
                                             self.real2draw_y(-x1 * math.sin(math.radians(-90)) +
                                                              y1 * math.cos(math.radians(-90))),
                                             self.real2draw_x(x2 * math.cos(math.radians(-90)) +
                                                              y2 * math.sin(math.radians(-90))),
                                             self.real2draw_y(-x2 * math.sin(math.radians(-90)) +
                                                              y2 * math.cos(math.radians(-90))), fill=colors[c])
                self.Filtree.create_rectangle(self.real2draw_x(0), self.real2draw_y(100),
                                              self.real2draw_x(0), self.real2draw_y(100), outline="yellow",
                                              width=10)
                c += 1
        except:
            pass
    # -----------------------------------------------------------------------------------------------
    def AffichageDelta(self):
        if len(VG.targetDistSum) > 1:
            self.DeltaScreen.delete("all")
            self.DeltaScreen.create_line(0, 200, 400, 200)
            self.DeltaScreen.create_line(1, 0, 1, 400)

            EchMax = max(VG.targetDistSum)
            EchMin = min(VG.targetDistSum)
            EchMax2 = max(VG.targetDistSum)
            EchMin2 = min(VG.targetDistSum)
            if abs(EchMin) > EchMax: EchMax = abs(EchMin)
            if abs(EchMin2) > EchMax2: EchMax2 = abs(EchMin2)
            if EchMax < 200: EchMax = 200
            if EchMax2 < 200: EchMax2 = 200
            EchMax = 200 / EchMax
            EchMax2 = 200 / EchMax2
            for i in range(len(VG.HistoriqueErreur) - 1):
                self.DeltaScreen.create_line(i, 200 - (VG.targetDistSum[i] * EchMax), i + 1,200 - (VG.targetDistSum[i + 1] * EchMax), fill='blue')
                #self.DeltaScreen.create_line(i, 200 - (VG.HistoriqueFiltre[i] * EchMax), i + 1,200 - (VG.HistoriqueFiltre[i + 1] * EchMax), fill='red')
            for i in range(len(VG.HistoriqueFiltre2) - 1):
                self.DeltaScreen.create_line(i, 200 - (VG.HistoriqueFiltre2[i] * EchMax), i + 1,200 - (VG.HistoriqueFiltre2[i + 1] * EchMax), fill='red')
            for i in range (len(VG.targetDistSum)-1):
                self.DeltaScreen.create_line(i , 200 - (VG.targetDistSum[i] * EchMax2), i + 1, 200 - (VG.targetDistSum[i + 1] * EchMax2), fill='green')
    # -----------------------------------------------------------------------------------------------
    def real2draw_x(self, x):
        step = self._width / float(self._sizeX)
        xdraw = x * float(step) + self._width / 2.0
        return xdraw
    # -----------------------------------------------------------------------------------------------
    def real2draw_y(self, y):
        step = self._height / float(self._sizeY)
        ydraw = y * float(step) + self._height / 2.0
        return self._height - ydraw
    # -----------------------------------------------------------------------------------------------
    # END OF DRAWING FUNCTIONS
    # -----------------------------------------------------------------------------------------------
    # Currently used functions for the Robot::
    # -----------------------------------------------------------------------------------------------
    def filtrage_initiale3(self):
        # Filtering initial LiDAR measurements by the distance between them and the robot.

        data_x_tmp = []
        data_y_tmp = []
        LOCK.acquire()  # lock the use of the LiDAR global variable to update the values
        for msr in LiDAR:
            if (3000 >= msr[0] >= 300) and not VG.averageDist: # : #
                data_x_tmp.append(float(msr[0] * math.sin(math.radians(msr[1] - 135 + 90))))
                data_y_tmp.append(float(msr[0] * math.cos(math.radians(msr[1] - 135 + 90))))
            elif (2500 >= msr[0] >= 300): # max(2000,(VG.averageDist * 2.5))
                data_x_tmp.append(float(msr[0] * math.sin(math.radians(msr[1] - 135 + 90))))
                data_y_tmp.append(float(msr[0] * math.cos(math.radians(msr[1] - 135 + 90))))
        LOCK.release()
        #print("data_x_tmp =\n",data_x_tmp,"\ndata_y_tmp =\n", data_y_tmp)
        return data_x_tmp, data_y_tmp
    # -----------------------------------------------------------------------------------------------
    def filtrage_initiale2(self):
        # Filtering the LiDAR measurements for the turning rows procedure, considering only that that is located at the
        # expected quadrant for that side of turning, also, disregarding points that are too close to the robot once
        # they may be from the row it just left, instead of the next one.
        data_x_tmp = []
        data_y_tmp = []

        LOCK.acquire()  # lock the use of the LiDAR global variable to update the values
        #measure = max((VG.averageDist/2.5),40)
        for msr in LiDAR:
            if (3000 >= msr[0] >= 300) and not VG.averageDist: # : #
                data_x_tmp.append(float(msr[0] * math.sin(math.radians(msr[1] - 135 + 90))))
                data_y_tmp.append(float(msr[0] * math.cos(math.radians(msr[1] - 135 + 90))))
            elif (2800 >= msr[0] >= min((VG.averageDist/1.4),2000)):
                data_x_tmp.append(float(msr[0] * math.sin(math.radians(msr[1] - 135 + 90))))
                data_y_tmp.append(float(msr[0] * math.cos(math.radians(msr[1] - 135 + 90))))
        LOCK.release()

        listY = []
        listX = []

        if VG.directionTurn == "Right":
            #print("Turning right")
            for i in range(len(data_x_tmp)):
                if (data_y_tmp[i] < 0):#(data_x_tmp[i] < 0) and
                    listX.append(data_x_tmp[i])
                    listY.append(data_y_tmp[i])
        else:
            #print("Turning Left")
            for i in range(len(data_x_tmp)):
                if (data_y_tmp[i] > 0):#(data_x_tmp[i] > 0) and
                    listX.append(data_x_tmp[i])
                    listY.append(data_y_tmp[i])

        VG.data_x_tmp = listX
        VG.data_y_tmp = listY

        #print("data_x_tmp =\n",data_x_tmp,"\ndata_y_tmp =\n", data_y_tmp)
        return listX, listY
    # -----------------------------------------------------------------------------------------------
    def box_filter2(self, data_x_tmp, data_y_tmp):
        # Filtering for the points at a perpendicular model from the robot, that is used when changing rows, this
        # allows the robot to narrow it's vision to the neighbourhood of the model that it is approaching to, this
        # function may require altering if we decide to follow a different strategy for row change (as in: going to the
        # next row not based on the average measudred width of the previous one, but instead really finding the next
        # row and attempting to aproach it.)
        approachFact = 10
        flag1 = False
        flag2 = False
        flag3 = False
        flag4 = False

        if VG.currentCRModel:

            model = VG.currentCRModel
            modelData = VG.currentCRModelData

            #print("modelData = ",modelData)
            #print("model = ",model)

            minx, maxx, miny, maxy = extrem_lists(modelData)

            LOCK.acquire()  # lock the use of the LiDAR global variable to update the values
            # measure = max((VG.averageDist/2.5),40)

            data_x_filt = []
            data_y_filt = []

            for msr in LiDAR:
                flag3 = False
                flag4 = False
                if (4000 >= msr[0] >= 300):  # : # and not VG.averageDist
                    y = float(msr[0] * math.sin(math.radians(msr[1] - 135 + 90)))
                    x = float(msr[0] * math.cos(math.radians(msr[1] - 135 + 90)))

                    coefficientTest = (float(model[0]) * x) + float(model[1] - approachFact) - y
                    """if ((maxx + abs(maxx/20)) >= x >= (minx - abs(minx/20))) and (((y > 0) and (y > miny)) or (((y < 0) and (y < maxy)))):# :
                        print("The extreme values are:\nX:\nmin = ", minx, "\nmax = ", maxx, "\nY:\nmin = ", miny,
                              "\nmax = ", maxy)
                        print("Added [",x,",", y,"]")
                        data_x_filt.append(x)
                        data_y_filt.append(y)"""
                    if (maxx < 0) and (x < maxx):
                        flag1 = True
                    elif (minx > 0) and (x > minx):
                        flag2 = True

                    if (VG.directionTurn == "Left") and (x > 0):
                        flag3 = True
                    elif (VG.directionTurn == "Right") and (x < 0):
                        flag4 = True

                    if (abs(coefficientTest) < 200) and (flag1 or flag2) and (flag3 or flag4):
                        #print("The extreme values are:\nX:\nmin = ", minx, "\nmax = ", maxx, "\nY:\nmin = ", miny,"\nmax = ", maxy)
                        #print("Added [", x, ",", y, "]")
                        data_x_filt.append(x)
                        data_y_filt.append(y)
            LOCK.release()

            VG.boxRight = [minx, maxx, miny, maxy]

            VG.data_x_tmp = data_y_filt
            VG.data_y_tmp = data_x_filt

            return data_y_filt, data_x_filt

        else:
            print("Fallen into else filterBox2, returning entry data")
            VG.boxRight = []
            return data_x_tmp, data_y_tmp
    # -----------------------------------------------------------------------------------------------
    def box_filter(self):
        # This functions filters the LiDAR measurements to fit the models that are currently being used to control
        # the robot, it allows for the reach of the robot's view to expand considerably at the directions where the
        # robot has models, so it eases the way for PEARL, giving it points that are extremely likely to belong to the
        # next models, in case of non existance of a model at one of the sides, (or both sides), the filter isn't used
        # and PEARL gets the raw readings for that side (or both).

        VG.data_x_tmp = []
        VG.data_y_tmp = []

        x_points = []
        y_points = []
        cloneMsr = []

        LOCK.acquire()

        for msr in range (len(LiDAR)):
            #print(msr)
            if (3000 >= LiDAR[msr][0] >= 300):  # : # and not VG.averageDist

                x_points.append(float(LiDAR[msr][0] * math.sin(math.radians(LiDAR[msr][1] - 135 + 90))))  # msr[0]*math.cos((msr[1]-135)*math.pi/180)#
                y_points.append(float(LiDAR[msr][0] * math.cos(math.radians(LiDAR[msr][1] - 135 + 90))))  # msr[0]*math.sin((msr[1]-135)*math.pi/180)#

                cloneMsr.append(LiDAR[msr])

        LOCK.release()

        if len(VG.currentModels) > 1:
            modelRight = [VG.currentModels[1][0],VG.currentModels[1][1]]
            modelLeft = [VG.currentModels[0][0],VG.currentModels[0][1]]
            modelRightData = VG.currentData[1]
            modelLeftData = VG.currentData[0]

        elif VG.currentModels[0][1] > 0:
            modelRight = [VG.currentModels[0][0],VG.currentModels[0][1]]
            modelRightData = VG.currentData[0]
            modelLeft = []
            modelLeftData = []

        elif VG.currentModels[0][1] < 0:
            modelLeft = [VG.currentModels[0][0],VG.currentModels[0][1]]
            modelLeftData = VG.currentData[0]
            modelRight = []
            modelRightData = []

        data_x_filt = []
        data_y_filt = []

        if (modelRight and (abs(modelRight[0]) < 0.15)):# and (VG.trueLeftHandModel and (abs(VG.trueLeftHandModel[0]) < 0.15)):
            minxR, maxxR, minyR, maxyR = extrem_lists(modelRightData)

            VG.boxRight = [minxR, maxxR, minyR, maxyR]

            #coefficientTestR = (float(modelRight[0]) * x) + float(modelRight[1]) - y
            #print(worstCaseR)

            rightThresh = 0.05 #* abs(modelRight[0])
            for i in range(len(x_points)):

                coefficientTestR = (float(modelRight[0]) * x_points[i]) + float(modelRight[1]) - y_points[i]

                if (abs(coefficientTestR) < 200) and (y_points[i] > 0):#((maxyR + abs(maxyR * rightThresh)) >= y_points[i] >= (minyR - abs(minyR * rightThresh))) and (((x_points[i] > 0) and (x_points[i] > (minxR - (minxR/10)))) or (((x_points[i] < 0) and (x_points[i] < (maxxR - (maxxR/10)))))):# ::#(abs(coefficientTest) < abs(worstCaseR)):#
                    #print("The extreme values are:\nX:\nmin = ", minx, "\nmax = ", maxx, "\nY:\nmin = ", miny,"\nmax = ", maxy)

                    data_x_filt.append(x_points[i])
                    data_y_filt.append(y_points[i])

            #print(modelRight,"\n",modelRightData,"\n",data_x_filt,"\n",data_y_filt)

        else:
            VG.boxRight =[]
            for i in range(len(cloneMsr)):
                if (3000 >= cloneMsr[i][0] >= 300) and not VG.averageDist:  # : #
                    #x = float(msr[0]*math.cos((msr[1]-135)*math.pi/180))#float(msr[0] * math.sin(math.radians(msr[1] - 135 + 90)))
                    #y = float(msr[0]*math.sin((msr[1]-135)*math.pi/180))#float(msr[0] * math.cos(math.radians(msr[1] - 135 + 90)))
                    data_x_filt.append(float(cloneMsr[i][0] * math.sin(math.radians(cloneMsr[i][1] - 135 + 90))))
                    data_y_filt.append(-float(cloneMsr[i][0] * math.sin(math.radians(cloneMsr[i][1] - 135 + 90))))
                elif ((VG.averageDist * 2.0) >= cloneMsr[i][0] >= 300):  # max(2000,(VG.averageDist * 2.5))
                    x = float(cloneMsr[i][0] * math.sin(math.radians(cloneMsr[i][1] - 135 + 90)))#msr[0]*math.cos((msr[1]-135)*math.pi/180)#
                    y = float(cloneMsr[i][0] * math.cos(math.radians(cloneMsr[i][1] - 135 + 90)))#msr[0]*math.sin((msr[1]-135)*math.pi/180)#
                    if (y > 0):
                        data_x_filt.append(x)
                        data_y_filt.append(y)

        #print("Added on the right:\nx:\n",data_x_filt,"\ny:\n",data_y_filt)

        if modelLeft and (abs(modelLeft[0]) < 0.15):

            minxL, maxxL, minyL, maxyL = extrem_lists(modelLeftData)

            VG.boxLeft = [minxL, maxxL, minyL, maxyL]

            worstCaseL = (float(modelLeft[0]) * minxL) + float(modelLeft[1]) - maxyL

            leftThresh = 0.05#8 * abs(modelLeft[0])
            """
            for i in range(len(x_points)):
                coefficientTest = (float(modelLeft[0]) * x_points[i]) + float(modelLeft[1]) - y_points[i]
                if ((maxyL + abs(maxyL * leftThresh)) >= y_points[i] >= (minyL - abs(minyL * leftThresh))) and (((x_points[i] > 0) and (x_points[i] > (minxL - (minxL/10)))) or (((x_points[i] < 0) and (x_points[i] < (maxxL - (maxxL/10)))))): # (abs(coefficientTest) < abs(worstCaseL)):#
                    #print("The extreme values are:\nX:\nmin = ", minx, "\nmax = ", maxx, "\nY:\nmin = ", miny,"\nmax = ", maxy)
                    #print("Added [",x,",", y,"]")
                    data_x_filt.append(x_points[i])
                    data_y_filt.append(y_points[i])

            #print(modelLeft,"\n",modelLeftData,"\n",data_x_filt,"\n",data_y_filt)"""


            for i in range(len(x_points)):

                coefficientTestL = (float(modelLeft[0]) * x_points[i]) + float(modelLeft[1]) - y_points[i]

                if (abs(coefficientTestL) < 200) and (y_points[i] < 0):#((maxyR + abs(maxyR * rightThresh)) >= y_points[i] >= (minyR - abs(minyR * rightThresh))) and (((x_points[i] > 0) and (x_points[i] > (minxR - (minxR/10)))) or (((x_points[i] < 0) and (x_points[i] < (maxxR - (maxxR/10)))))):# ::#(abs(coefficientTest) < abs(worstCaseR)):#
                    #print("The extreme values are:\nX:\nmin = ", minx, "\nmax = ", maxx, "\nY:\nmin = ", miny,"\nmax = ", maxy)

                    data_x_filt.append(x_points[i])
                    data_y_filt.append(y_points[i])


        else:
            VG.boxLeft = []
            for i in range(len(cloneMsr)):
                if (3000 >= cloneMsr[i][0] >= 300) and not VG.averageDist:  # : #
                    #x = float(msr[0]*math.cos((msr[1]-135)*math.pi/180))#float(msr[0] * math.sin(math.radians(msr[1] - 135 + 90)))
                    #y = float(msr[0]*math.sin((msr[1]-135)*math.pi/180))#float(msr[0] * math.cos(math.radians(msr[1] - 135 + 90)))
                    data_x_filt.append(float(cloneMsr[i][0] * math.sin(math.radians(cloneMsr[i][1] - 135 + 90))))
                    data_y_filt.append(-float(cloneMsr[i][0] * math.sin(math.radians(cloneMsr[i][1] - 135 + 90))))
                elif ((VG.averageDist * 2.0) >= cloneMsr[i][0] >= 300):  # max(2000,(VG.averageDist * 2.5))
                    x = float(cloneMsr[i][0] * math.sin(math.radians(cloneMsr[i][1] - 135 + 90)))#msr[0]*math.cos((msr[1]-135)*math.pi/180)#
                    y = float(cloneMsr[i][0] * math.cos(math.radians(cloneMsr[i][1] - 135 + 90)))#msr[0]*math.sin((msr[1]-135)*math.pi/180)#
                    if (y < 0):
                        data_x_filt.append(x)
                        data_y_filt.append(y)
        LOCK.acquire()
        #print("Acquired data at this iteration:")
        #print("data_x_filt: ",data_x_filt,"data_y_filt = " ,data_y_filt)
        VG.data_x_tmp = data_x_filt
        VG.data_y_tmp = data_y_filt
        LOCK.release()
        return data_x_filt, data_y_filt
    # -----------------------------------------------------------------------------------------------
    def FiltreRIF2(self, Coef):
        # RIF Filter - Filter with a finite response to the impulse, in order to weight the history of errors when
        # calculating the Delta that will be used for the PID controller.
        #-#print("Rodando filtreRIF")
        #VG.targetDistSum[(len(VG.targetDistSum) - 1)]
        Delta = 0
        for item in range(len(Coef)):
            if item >= len(VG.targetDistSum):
                break
            else:
                Delta = Delta + (VG.targetDistSum[len(VG.targetDistSum)-1-item] * Coef[item])
                # print("Delta filtré " + str(item) + " : " + str(Delta))
        VG.HistoriqueFiltre2.append(Delta)
        #print("Delta in FiltreRIF =",Delta)
        return Delta
    # -----------------------------------------------------------------------------------------------
    def Correcteur3(self, Delta, Droite_dInteret, Coef_P, Coef_I, Coef_D, Vitesse_execution):
        # Calculates the the control input that will be needed at the robot's wheels, it's a dynamical way of
        # calculating, considering the angle theta between the robot and the rows and, if no model is present for
        # controlling, it also uses the angle as control input.

        #-#print("Rodando Correcteur")
        #-#print("Delta in Correcteur =", Delta)
        # VG.HistoriqueErreur.append(Delta)

        if (abs(VG.theta) > 10):
            # If there's no model to follow, reduce speed, give time to the robot to find the model.
            print("Alternative Control")
            #Vmax = (VG.speedTerminal * Vitesse_execution / 240)
            Vmax = (VG.speedTerminal * Vitesse_execution / 100)
            Delta = VG.theta
            param_P = Delta * 20 *(Coef_P)
            """Delta = 5 * VG.historyAngleX[len(VG.historyAngleX) - 1]
            param_P = Delta * Coef_P
            param_I = (5*sum(VG.historyAngleX)) * Coef_I
            param_D = (Delta - (5*VG.historyAngleX[len(VG.historyAngleX) - 2])) * Coef_D
            Commande = (param_P) + (param_I) + (param_D)"""
            Commande = (param_P) # PENDANT qu'on ne recupère pas les mesures du accel du robot
            #%%#print("## CORRECTEUR Angle: Delta : " + str(Delta) + "\tCommande : " + str(Commande) + "\tparam_P : " + str(param_P) + "\tparam_I : " + str(param_I) + "\tparam_D : " + str(param_D))

            if Commande < 0:
                Data_Moteurs = [Vmax - abs(Commande), Vmax]
            else:
                Data_Moteurs = [Vmax , Vmax - abs(Commande)]

            print("Data_Moteurs = ",Data_Moteurs)
            print("Commande = ",Commande)

        elif not Droite_dInteret:
            Vmax = (VG.speedTerminal * (Vitesse_execution/1.5) / 100)
            print("No models")
            param_P = Delta * Coef_P
            param_I = sum(VG.HistoriqueFiltre2) * Coef_I
            param_D = (Delta - VG.HistoriqueFiltre2[len(VG.HistoriqueFiltre2) - 2]) * Coef_D
            Commande = (param_P) + (param_I) + (param_D)
            if Commande < 0:
                Data_Moteurs = [Vmax - abs(Commande), Vmax]
            else:
                Data_Moteurs = [Vmax , Vmax - abs(Commande)]

            print("Data_Moteurs = ",Data_Moteurs)
            print("Commande = ",Commande)

        else:
            param_P = Delta * Coef_P
            param_I = sum(VG.HistoriqueFiltre2) * Coef_I
            param_D = (Delta - VG.HistoriqueFiltre2[len(VG.HistoriqueFiltre2) - 2]) * Coef_D
            Commande = (param_P) + (param_I) + (param_D)

            #%%#print("## PEARL: Delta : " + str(Delta) + "\tCommande : " + str(Commande) + "\tparam_P : " + str(param_P) + "\tparam_I : " + str(param_I) + "\tparam_D : " + str(param_D))
            Vmax = (VG.speedTerminal * Vitesse_execution / 100)

            if len(Droite_dInteret) > 0:
                if (Droite_dInteret[0][1] > 0 and Commande > 0) or (Droite_dInteret[0][1] < 0 and Commande < 0):
                    Data_Moteurs = [Vmax, (Vmax - abs(Commande))]
                else:
                    Data_Moteurs = [(Vmax - abs(Commande)), Vmax]
            else:
                Data_Moteurs = [0.05 * Vitesse_execution, 0.05 * Vitesse_execution]

        """"if not (Droite_dInteret):

            Data_Moteurs[1] = 10
            Data_Moteurs[0] = 10"""

        if Data_Moteurs[0] < 10:
            Data_Moteurs[0] = 10
        if Data_Moteurs[1] < 10:
            Data_Moteurs[1] = 10

        return Data_Moteurs
    # -----------------------------------------------------------------------------------------------
    def find_lines(self):
        # Function that hosts the PEARL method, and is also the center part of the whole program's execution, it is
        # constantly called during the passage between rows and when changing rows, the find_lines2 function is called
        # which is essentially the same function, but with the coordinates x and y inverted in order to ease
        # calculations of straight line models following the y = ax + b equation.

        Energy = float('inf')
        newEnergy = 0
        flagEnergy = True

        # Selecting the methodology that will be used for obtaining the data:
        if not (VG.trueRightHandModel or VG.trueLeftHandModel):  # (VG.currentModels)
            x_data, y_data = self.filtrage_initiale3()
        else:
            x_data, y_data = self.box_filter()
        # &#print("Entering the PEARL METHOD")

        # Entering PEARL
        if (len(x_data) != 0):
            # The number of initial model is not a constant, it adapts to the amount of points we have.
            nbOfInitModels = int(len(x_data) / 8)  # 6#
            Clustered_data = []
            Clustered_data_lines = []
            Data_dInteret = []
            Droite_dInteret = []

            if nbOfInitModels:
                nbMinPoints = int(len(x_data) / 8)
                labels = np.zeros(len(x_data))

                # Initialization setting the initial models L0 and the model of outliers.
                labels, models = self.initialStep2(x_data, y_data, labels, nbOfInitModels)

                # First computation of alpha expansion
                models, labels, newEnergy = self.expansionEnergy(x_data, y_data, labels, models)

                # print("Models before first modelShrink =",models)
                models, labels = self.modelsShrinkInit(x_data, y_data, labels, models, int(1.5 * nbMinPoints))

                # Saving models from the earlier iteration (in case the energy does not decrease)
                savedModels, savedLabels = self.saveModelsLabels(models, labels)

                nbOfIteractions = 0

                while (nbOfIteractions < VG.nbMaxIteractions):

                    # Searching for new models amongst the points that are currently at the outliers pool:
                    models, labels = self.searchNewModels2(x_data, y_data, labels, models)

                    # Remodeling the system in order to have a more accurate model:
                    models, labels = self.reEstimation2(x_data, y_data, labels, models)

                    # Destroying models that are incoherent with the rest:
                    models, labels = self.modelsKill3(labels, models, 3 * VG.threshRatio)

                    # Merging models that are too close to each other:
                    models, labels = self.modelsMS2(models, labels)

                    # Added part, looking for models that are actually parallel
                    models, labels, newEnergy = self.expansionEnergy(x_data, y_data, labels, models)

                    if (newEnergy > Energy) or (nbOfIteractions == VG.nbMaxIteractions):
                        # If any of the two conditions were met, it's the end of the procedure
                        break

                    Energy = float(newEnergy)

                    # Saving this iteractions models in case the energy does not decrease at the next one.
                    savedModels, savedLabels = self.saveModelsLabels(models, labels)


                    nbOfIteractions = nbOfIteractions + 1

                # print("The End of PEARL")

                # Amongst the final models, we find the min and max x coordinates for the points in each of them and
                # create sets that will have all of the points of that model.
                for model in range(0, len(savedModels)):
                    data_model = []
                    minx = float('inf')
                    maxx = float('-inf')
                    for label in range(0, len(savedLabels)):
                        if (savedLabels[label] == savedModels[model][2]):
                            data_model.append([x_data[label], y_data[label]])
                            # print("Clustered_data =",Clustered_data)
                            if (x_data[label] < minx):
                                minx = x_data[label]
                            if (x_data[label] > maxx):
                                maxx = x_data[label]

                    Clustered_data.append(data_model)
                    Clustered_data_lines.append([savedModels[model][0], savedModels[model][1], minx, maxx])

                # Data_dInteret, Droite_dInteret, TypeRang = self.filtrage_droites5(Clustered_data, Clustered_data_lines)

                # We select the filtering that will be used for the straight lines (rows) based on the flag that states
                # if the robot is falling back or not. For the moment, this is not used for controlling or decision
                # making, once the fall back procedure is made using the IMU data. In case change on that aspect is
                # needed (or even improvement), the filtrage_droitesFlex2 is the path to follow, tweaking it's
                # coefficients in order to allow the filter to be more flexible and actually helps the robot find the
                # rows that it's supposed to.
                if not VG.fallBack:
                    Data_dInteret, Droite_dInteret, TypeRang = self.filtrage_droitesFlex(Clustered_data,
                                                                                     Clustered_data_lines)
                else:
                    Data_dInteret, Droite_dInteret, TypeRang = self.filtrage_droitesFlex2(Clustered_data,
                                                                                     Clustered_data_lines)

                # If we are under the follow row regime (set by the flag parcoursRangee), the motors speeds are defined
                # at this step, using an RIF filter and a PID controller.
                if VG.ParcoursRangee:
                    print("-------------------------------------------------------------------------------------------")
                    print("Running Automatically;" + "\tFollowing model: : " + str(TypeRang))
                    print("IMU = ", VG.IMUStatus, "Theta2 = ", float(VG.theta), "(posx, posy) = (", VG.posx, ",", VG.posy, ")")

                    # Delta = self.FiltreRIF([5 / 10, 1 / 10, 3 / 10])
                    # Data_moteurs = self.Correcteur(Delta, Droite_dInteret, 0.2, 0, 0.9, 90)

                    Delta = self.FiltreRIF2([8 / 10, 1 / 10, 1 / 10])

                    # Data_moteurs = self.Correcteur2(Delta, Droite_dInteret, 0.12, 0, 0.2, VG.speedMax)
                    Data_moteurs = self.Correcteur3(Delta, Droite_dInteret, 0.15, 0, 0.07, VG.speedMax)

                    VG.speedRightMotor = Data_moteurs[1]  # int(Data_moteurs[1] / 2)
                    VG.speedLeftMotor = Data_moteurs[0]  # int(Data_moteurs[0] / 2)

                # print("Clustered Data =", Clustered_data)
                # print("Clustered Data_Lines", Clustered_data_lines)
                self.draw(Clustered_data, Clustered_data_lines)
                # print("Data_dInteret = ",Data_dInteret," \nDroite_dInteret = " ,Droite_dInteret)
                self.draw2(Data_dInteret, Droite_dInteret)

                # We save the models that we are effectively following (the filter may retain models that it
                # won't give as possible following models. This is due to the characteristics of the filter.)
                VG.currentModels = Droite_dInteret
                VG.currentData = Data_dInteret

                # self.plotAll(models, labels, x_data, y_data, nbOfInitModels, nbOfIteractions, savedModels, savedLabels)

                # self.AffichageDelta()
                # self.AffichageDelta2()
                # self.AffichageDelta3()
                # self.distanceCalculation()
                # self.angleCalculation()
                # self.accurageAngleCalculation()

                # self.AffichageAccel()
                # print("\nnbOfIteractions of this Step =",nbOfIteractions)

                # If we detect that we are at the row change procedure, we call the second variant PEARL method, which
                # uses the same procedure as this one, but with small changes to the filters.
                if VG.turningRow:
                    # &#print("Calling PEARL 2")
                    self.after(100, self.find_lines2)
                    VG.trueRightHandModel = []
                    VG.trueLeftHandModel = []
                    VG.trueRightModelData = []
                    VG.trueLeftModelData = []
                else:
                    self.after(100, self.find_lines)
                    ##print("Re-calling PEARL1 in normal condition")

            else:
                # print("########################################## FIN DE RANG PEARL")
                # If we have a smaller amount of data points than it would be necessary to build a single model (or
                # even 0 readings), we've reached the end of the row, this step initiates the row change procedure in
                # which the automatic_mode and turning functions will take control of the motors.

                if VG.flagFirst:

                    VG.flagInitModels = True
                    VG.ParcoursRangee = False
                    VG.repositionOnRow = True
                    VG.lenRows.append(VG.spaceX)
                    VG.yRows.append(VG.spaceY)

                    VG.spaceX = 0
                    VG.spaceY = 0

                    VG.flagFirst = False

                    self.draw(Clustered_data, Clustered_data_lines)
                    self.draw2(Data_dInteret, Droite_dInteret)
                else:
                    self.draw(Clustered_data, Clustered_data_lines)
                    self.draw2(Data_dInteret, Droite_dInteret)
                # Understand why it never enters in PEARL 2
                if VG.turningRow:
                    # &#print("Calling PEARL 2")
                    VG.trueRightHandModel = []
                    VG.trueLeftHandModel = []
                    VG.trueRightModelData = []
                    VG.trueLeftModelData = []
                    self.after(100, self.find_lines2)
                else:
                    ##print("Re-calling PEARL1 after row end")
                    self.after(100, self.find_lines)


        else:
            ##print("No Readings made...")
            if VG.turningRow:
                # &#print("Calling PEARL 2")
                VG.trueRightHandModel = []
                VG.trueLeftHandModel = []
                VG.trueRightModelData = []
                VG.trueLeftModelData = []
                self.after(100, self.find_lines2)
            else:
                ##print("Re-calling PEARL1 without readings")
                if (len(VG.lineHistory)):
                    if VG.flagFirst:
                        VG.flagInitModels = True
                        VG.ParcoursRangee = False
                        VG.repositionOnRow = True
                        VG.lenRows.append(VG.spaceX)
                        VG.yRows.append(VG.spaceY)

                        VG.spaceX = 0
                        VG.spaceY = 0

                        VG.flagFirst = False


                    if VG.turningRow:
                        # &#print("Calling PEARL 2")
                        VG.trueRightHandModel = []
                        VG.trueLeftHandModel = []
                        VG.trueRightModelData = []
                        VG.trueLeftModelData = []
                        self.after(100, self.find_lines2)
                    else:
                        ##print("Re-calling PEARL1 after row end")
                        self.after(100, self.find_lines)
                else:
                    self.after(100, self.find_lines)
    # -----------------------------------------------------------------------------------------------
    def find_lines2(self):
        # Responsible for finding perpendicular rows when the robot is changing rows. Essentially the same as
        # find_lines, but calling different filters for the results of PEARL
        # print("\n#################\n\nRodando find_lines 1")
        Energy = float('inf')
        newEnergy = 0
        flagEnergy = True

        # Inverting x and y data in order to get the correct models in a perpendicular basis

        y_data, x_data = self.filtrage_initiale2()

        if VG.currentCRModel and VG.searchingRow: # :#
            y_data, x_data = self.box_filter2(y_data, x_data)


        #-#print("Entering the PEARL METHOD 2")
        if (len(x_data) != 0):

            nbOfInitModels = int(len(x_data) / 8)  # 6#
            Clustered_data = []
            Clustered_data_lines = []
            Data_dInteret = []
            Droite_dInteret = []

            if nbOfInitModels:
                nbMinPoints = int(len(x_data) / 8)
                labels = np.zeros(len(x_data))

                # Initialization setting the initial models L0 and the model of outliers.
                labels, models = self.initialStep2(x_data, y_data, labels, nbOfInitModels)
                models, labels, newEnergy = self.expansionEnergy(x_data, y_data, labels, models)
                models, labels = self.modelsShrinkInit(x_data, y_data, labels, models, int(1.5 * nbMinPoints))
                savedModels, savedLabels = self.saveModelsLabels(models, labels)
                nbOfIteractions = 0

                while (nbOfIteractions < VG.nbMaxIteractions):

                    models, labels = self.searchNewModels2(x_data, y_data, labels, models)
                    # Remodeling the system in order to have a more accurate model:
                    models, labels = self.reEstimation2(x_data, y_data, labels, models)
                    models, labels = self.modelsKill3(labels, models, 3 * VG.threshRatio)
                    # Merging models (optional step)
                    models, labels = self.modelsMS2(models, labels)
                    # Added part, looking for models that are actually parallel
                    models, labels, newEnergy = self.expansionEnergy(x_data, y_data, labels, models)
                    if (newEnergy > Energy) or (nbOfIteractions == VG.nbMaxIteractions):
                        break
                    Energy = float(newEnergy)
                    savedModels, savedLabels = self.saveModelsLabels(models, labels)

                    nbOfIteractions = nbOfIteractions + 1

                for model in range(0, len(savedModels)):
                    data_model = []
                    minx = float('inf')
                    maxx = float('-inf')
                    for label in range(0, len(savedLabels)):
                        if (savedLabels[label] == savedModels[model][2]):
                            data_model.append([x_data[label], y_data[label]])
                            if (x_data[label] < minx):
                                minx = x_data[label]
                            if (x_data[label] > maxx):
                                maxx = x_data[label]

                    Clustered_data.append(data_model)
                    Clustered_data_lines.append([savedModels[model][0], savedModels[model][1], minx, maxx])

                # Data_dInteret, Droite_dInteret, TypeRang = self.filtrage_droites(Clustered_data, Clustered_data_lines)
                # Data_dInteret, Droite_dInteret, TypeRang = self.filtrage_droites3(Clustered_data,Clustered_data_lines)  #
                Data_dInteret, Droite_dInteret, TypeRang = self.filtrage_droites6(Clustered_data,Clustered_data_lines)

                self.draw(Clustered_data, Clustered_data_lines)

                self.draw2(Data_dInteret, Droite_dInteret)

                print("IMU = ", VG.IMUStatus, "| Theta2 = ", float(VG.theta), "| (posx, posy) = (", VG.posx, ",", VG.posy,")")

                if Droite_dInteret:
                    VG.currentCRModel = Droite_dInteret[0]
                    VG.currentCRModelData = Data_dInteret[0]

                # self.AffichageDelta()
                # self.AffichageDelta2()
                # self.AffichageDelta3()

                if VG.fallBack:
                    #-#print("Finished turn, leaving PEARL 2")
                    #VG.flagChangeRow = False
                    VG.trueRightHandModel = []
                    VG.trueLeftHandModel = []
                    VG.trueRightModelData = []
                    VG.trueLeftModelData = []
                    VG.flagInitModels2 = True
                    VG.flagInitModels3 = True
                    #VG.flagInitModels = True
                    self.after(100, self.find_lines)
                else:
                    if VG.flagWarning:
                        Data_dInteret = []
                        Droite_dInteret = []
                    self.after(100, self.find_lines2)

            else:

                #-#print("########################################## FIN DE RANG PEARL2")
                VG.flagInitModels = True
                #VG.ParcoursRangee = False
                #VG.repositionOnRow = True

                self.draw(Clustered_data, Clustered_data_lines)
                self.draw2(Data_dInteret, Droite_dInteret)

                if VG.fallBack:
                    #&#print("Finished turn, leaving PEARL 2")
                    VG.trueRightHandModel = []
                    VG.trueLeftHandModel = []
                    VG.trueRightModelData = []
                    VG.trueLeftModelData = []
                    #VG.flagChangeRow = False
                    VG.flagInitModels2 = True
                    VG.flagInitModels3 = True
                    #VG.flagInitModels = True
                    self.after(100, self.find_lines)
                else:
                    if VG.flagWarning:
                        Data_dInteret = []
                        Droite_dInteret = []
                    self.after(100, self.find_lines2)

        else:
            #&#print("No Readings made...")
            #-#print("VG.flagChangeRow =", VG.flagUsePearl2)
            if VG.fallBack:
                #&#print("Finished turn, leaving PEARL 2")
                VG.trueRightHandModel = []
                VG.trueLeftHandModel = []
                VG.trueRightModelData = []
                VG.trueLeftModelData = []
                # VG.flagChangeRow = False
                VG.flagInitModels2 = True
                VG.flagInitModels3 = True
                #VG.flagInitModels = True
                self.after(100, self.find_lines)

            else:
                if VG.flagWarning:
                    Data_dInteret = []
                    Droite_dInteret = []
                if VG.searchingRow:# and not VG.currentCRModel:
                    #&#print("I was searching for a newRow but found no readings, I've passed the end of the last row")
                    VG.searchingRow = False
                    VG.flagLastRow = True
                self.after(100, self.find_lines2)
    # -----------------------------------------------------------------------------------------------
    def filtrage_droitesFlex(self, Clustered_data, Clustered_data_lines):
        # Filtering the results of PEARL based on the previous model we were following (or had, but weren't following,
        # in some cases). It compares the set of models that arrived with the two main models we had before (Right
        # and/or left) in order to come up with the best possible model for the robot to follow. A safety mechanism was
        # added that prevents the rejection of too many PEARL results in a row, it consists of a counter of permanence
        # that erases the trueModels that we had if 12 successive results of PEARL are rejected, after that rejection,
        # the filter becomes more flexbile (as it goes into initialization mode) and is more likely to accept models
        # coming from PEARL. This is important because in case of a fast correction, the resulting models may diverge
        # from the previous ones by more than the tolerance given by the strict filters, and in that case, the robot
        # would never have a new mode, and consequently would run over the plants.
        min1B = float('-inf')
        min2B = float('inf')
        rightHandModel = []
        leftHandModel = []
        leLeftModels = []
        leRightModels = []
        Droite_dInteret = []
        Data_dInteret = []
        # print("Clustered_data_lines = ",Clustered_data_lines)

        for k in range(len(Clustered_data_lines)):
            # print("Clustered_data[k][2] = ",Clustered_data_lines[k][1])
            if (Clustered_data_lines[k][1] < 0):
                leLeftModels.append(Clustered_data_lines[k])
            if (Clustered_data_lines[k][1] > 0):
                leRightModels.append(Clustered_data_lines[k])

            if (Clustered_data_lines[k][1] > min1B) and (Clustered_data_lines[k][1] < 0):
                min1B = Clustered_data_lines[k][1]
                leftHandModel = Clustered_data_lines[k]
                # leLeftModels.append(Clustered_data_lines[k])
                index2 = k
            if (Clustered_data_lines[k][1] < min2B) and (Clustered_data_lines[k][1] > 0):
                min2B = Clustered_data_lines[k][1]
                rightHandModel = Clustered_data_lines[k]
                # leRightModels.append(Clustered_data_lines[k])
                index = k

        if VG.counterPermanence > 12:
            # print("The models that arrived were:\nleftHandModel =",leftHandModel,"\nrightHandModel =",rightHandModel)
            # print("we had\nTrueLeftHandModel =",VG.trueLeftHandModel,"\nTrueRightHandModel =",VG.trueRightHandModel)
            VG.flagInitModels = True
            VG.trueRightHandModel = []
            VG.trueLeftHandModel = []
            VG.trueRightModelData = []
            VG.trueLeftModelData = []
            VG.counterPermanence = 0
            #-#print("VG.trueRightHandModel =", VG.trueRightHandModel, "\nVG.trueLeftHandModel =", VG.trueLeftHandModel)
            #-#print("VG.lineHistory =\n", VG.lineHistory)
            #-#print("The averages for the coefficients are:\nrightHandAverage =", VG.rightHandAverage,"\nleftHandAverage =", VG.leftHandAverage)

        if VG.flagInitModels and rightHandModel and leftHandModel:
            #&#print("On Initialization both models...  ADDED SAFETY FOR INITIALIZATION ONLY WITH PERFECT MODEL")
            if rightHandModel and leftHandModel and (abs(rightHandModel[0] - leftHandModel[0]) <= 0.5) and not ((VG.rightHandAverage or VG.leftHandAverage) and (VG.averageDistToLeft or VG.averageDistToRight)) and not VG.averageDist and (abs(rightHandModel[0]) < 0.2):
                VG.trueRightHandModel = rightHandModel
                VG.trueLeftHandModel = leftHandModel
                VG.trueRightModelData = Clustered_data[index]
                VG.trueLeftModelData = Clustered_data[index2]
                VG.counterPermanence = 0
                VG.flagInitModels = False
            else:
                #-#print("initializing with average values already set")
                dist1 = abs(leftHandModel[1] - rightHandModel[1])
                if (abs(leftHandModel[0] - rightHandModel[0]) <= 0.2) and (abs(leftHandModel[0] - VG.leftHandAverage) <= 0.5) and (abs(rightHandModel[0] - VG.rightHandAverage) <= 0.5) and (VG.averageDist and (abs((VG.averageDist - dist1)/VG.averageDist) < 0.5)):  # and ((abs(dist1 - dist2)/dist2) < 0.7): #and (abs(VG.trueLeftHandModel[0] - leftHandModel[0]) <= 0.5) and (abs(VG.trueRightHandModel[0] - rightHandModel[0]) <= 0.5): # : # and VG.flagParallels:
                    # print("The new models are parallel, change will happen...")
                    VG.trueLeftHandModel = leftHandModel
                    VG.trueLeftModelData = Clustered_data[index2]
                    VG.trueRightHandModel = rightHandModel
                    VG.trueRightModelData = Clustered_data[index]
                    VG.counterPermanence = 0
                    VG.flagInitModels = False
        # Mark

        elif rightHandModel and leftHandModel and not VG.flagInitModels:
            #&#print("Entered: rightHandModel and leftHandModel")  # the Conditions after ((abs(rightHandModel[1] - VG.trueRightHandModel[1])/rightHandModel[1]) < 0.3) were added after the video niveau3

            if VG.trueRightHandModel and VG.trueLeftHandModel:
                # print("I have true models for both sides!")
                dist1 = abs(leftHandModel[1] - rightHandModel[1])
                dist2 = abs(VG.trueLeftHandModel[1] - VG.trueRightHandModel[1])
                if (abs(leftHandModel[0] - rightHandModel[0]) <= 0.2) and ((abs(leftHandModel[0] - VG.leftHandAverage) <= 0.4) or (abs(VG.trueLeftHandModel[0] - leftHandModel[0]) <= 0.7)) and ((abs(rightHandModel[0] - VG.rightHandAverage) <= 0.4) or (abs(VG.trueRightHandModel[0] - rightHandModel[0]) <= 0.7)) and (abs((VG.averageDist - dist1)/VG.averageDist) < 0.5):  # and ((abs(dist1 - dist2)/dist2) < 0.7): #and (abs(VG.trueLeftHandModel[0] - leftHandModel[0]) <= 0.5) and (abs(VG.trueRightHandModel[0] - rightHandModel[0]) <= 0.5): # : # and VG.flagParallels:
                    # print("The new models are parallel, change will happen...")
                    VG.trueLeftHandModel = leftHandModel
                    VG.trueLeftModelData = Clustered_data[index2]
                    VG.trueRightHandModel = rightHandModel
                    VG.trueRightModelData = Clustered_data[index]
                    VG.counterPermanence = 0
                else:
                    ##print("Not enough parallel!!!!\nKeeping old models!!!!")
                    # print("models are:\nVG.trueRightHandModel =",VG.trueRightHandModel,"\nVG.trueLeftHandModel =",VG.trueLeftHandModel,"\nRightHandModel =",rightHandModel,"\nleftHandModel",leftHandModel)
                    VG.counterPermanence = VG.counterPermanence + 1
                    # Data_dInteret = [VG.trueLeftModelData, VG.trueRightModelData]
                    # Droite_dInteret = [VG.trueLeftHandModel, VG.trueRightHandModel]
            else:
                # print("I had only one side of trueModels!!")

                # print("The models are:\nleftHandModel = ",leftHandModel,"\nAnd rightHandModel =",rightHandModel)

                if VG.trueLeftHandModel and not VG.trueRightHandModel:
                    # print("I had a leftHandModel but not a right hand one")
                    dist1 = abs(leftHandModel[1] - rightHandModel[1])
                    if (abs(leftHandModel[0] - rightHandModel[0]) <= 0.2) and ((abs(VG.trueLeftHandModel[0] - leftHandModel[0]) <= 1.3) or (abs(VG.leftHandAverage - leftHandModel[0]) <= 0.7)) and (abs((VG.averageDist - dist1)/VG.averageDist) < 0.5):
                        # print("The condition (abs(leftHandModel[0] - rightHandModel[0]) <= 0.2) and (abs(VG.trueLeftHandModel - leftHandModel[0]) <= 0.8) was satisfied, changing models...")
                        VG.trueLeftHandModel = leftHandModel
                        VG.trueLeftModelData = Clustered_data[index2]
                        VG.trueRightHandModel = rightHandModel
                        VG.trueRightModelData = Clustered_data[index]
                        Data_dInteret = [VG.trueLeftModelData, VG.trueRightModelData]
                        Droite_dInteret = [VG.trueLeftHandModel, VG.trueRightHandModel]
                        VG.lineHistory.append([Droite_dInteret[0][0], Droite_dInteret[0][1], Droite_dInteret[1][0],Droite_dInteret[1][1]])
                        VG.counterPermanence = 0

                    else:
                        # print("The big condition wasn't satisfied, so I'm keeping the old model.")
                        Data_dInteret = [VG.trueLeftModelData]
                        Droite_dInteret = [VG.trueLeftHandModel]
                        VG.lineHistory.append([Droite_dInteret[0][0], Droite_dInteret[0][1], None, None])
                        VG.counterPermanence = VG.counterPermanence + 1

                elif VG.trueRightHandModel and not VG.trueLeftHandModel:
                    # print("I had a rightHandModel but not a left hand one")
                    dist1 = abs(leftHandModel[1] - rightHandModel[1])
                    if (abs(leftHandModel[0] - rightHandModel[0]) <= 0.2) and ((abs(VG.trueRightHandModel[0] - rightHandModel[0]) <= 1.3) or (abs(VG.rightHandAverage - rightHandModel[0]) <= 0.7)) and (abs((VG.averageDist - dist1)/VG.averageDist) < 0.5):
                        # print("The condition (abs(leftHandModel[0] - rightHandModel[0]) <= 0.2) and (abs(VG.trueRightHandModel[0] - rightHandModel[0]) <= 0.8)   was satisfied, changing models...")
                        VG.trueLeftHandModel = leftHandModel
                        VG.trueLeftModelData = Clustered_data[index2]
                        VG.trueRightHandModel = rightHandModel
                        VG.trueRightModelData = Clustered_data[index]
                        Data_dInteret = [VG.trueLeftModelData, VG.trueRightModelData]
                        Droite_dInteret = [VG.trueLeftHandModel, VG.trueRightHandModel]
                        VG.lineHistory.append([Droite_dInteret[0][0], Droite_dInteret[0][1], Droite_dInteret[1][0],
                                               Droite_dInteret[1][1]])
                        VG.counterPermanence = 0
                    else:
                        # print("The big condition2 wasn't satisfied, so I'm keeping the old model.")
                        Data_dInteret = [VG.trueRightModelData]
                        Droite_dInteret = [VG.trueRightHandModel]
                        VG.lineHistory.append([None, None, Droite_dInteret[0][0], Droite_dInteret[0][1]])
                        VG.counterPermanence = VG.counterPermanence + 1

        elif leRightModels and not leLeftModels:# and not VG.flagInitModels:#
            #&#print("Entered: rightHandModels and (not leLeftModels)")
            # print("The true models are:\nleft Hand = ", VG.trueLeftHandModel, "\nright Hand = ", VG.trueRightHandModel)

            flagFoundModelsRight = False

            for k in range(len(leRightModels)):
                # print("Executing the for...", k)
                if (abs(rightHandModel[0] - leRightModels[k][0]) <= 0.1) and (rightHandModel != leRightModels[k]):
                    # print("I have two parallel Models in leRightModels, they are:",rightHandModel,"and\n",leRightModels[k])
                    flagFoundModelsRight = True

            if VG.flagInitModels and ((abs(rightHandModel[0] - VG.rightHandAverage) <= 0.15) or flagFoundModelsRight):#flagFoundModelsRight:
                ##print("On Initialization")
                if VG.averageDist and ((abs((VG.averageDist/2) - rightHandModel[1])/(VG.averageDist/2)) < 1):
                    #-#print("New model is close enough!")
                    VG.trueRightHandModel = rightHandModel
                    VG.trueRightModelData = Clustered_data[index]
                    VG.trueLeftHandModel = []
                    VG.trueLeftModelData = []
                    Data_dInteret = [VG.trueRightModelData]
                    Droite_dInteret = [VG.trueRightHandModel]
                    VG.lineHistory.append([None, None, Droite_dInteret[0][0], Droite_dInteret[0][1]])
                    VG.counterPermanence = 0
                    VG.flagInitModels = False
                elif VG.averageDist and not ((abs((VG.averageDist/2) - rightHandModel[1])/(VG.averageDist/2)) < 1):
                    #-#print("2New model is really far")
                    #-#print("rightHandModel[1] = ",rightHandModel[1])
                    #-#print("(abs((VG.averageDist/2) - rightHandModel[1]/(VG.averageDist/2)) = ",(abs((VG.averageDist/2) - rightHandModel[1])/(VG.averageDist/2)))
                    VG.flagInitModels = True
                else:
                    print("No mean yet, not initializing with single Right model.")
                    """VG.trueRightHandModel = rightHandModel
                    VG.trueRightModelData = Clustered_data[index]
                    VG.trueLeftHandModel = []
                    VG.trueLeftModelData = []
                    Data_dInteret = [VG.trueRightModelData]
                    Droite_dInteret = [VG.trueRightHandModel]
                    VG.lineHistory.append([None, None, Droite_dInteret[0][0], Droite_dInteret[0][1]])
                    VG.counterPermanence = 0"""
                    VG.flagInitModels = True

            elif flagFoundModelsRight and VG.trueRightHandModel:
                if VG.trueLeftHandModel and (abs(VG.trueLeftHandModel[0] - rightHandModel[0]) <= 0.2):
                    VG.trueRightHandModel = rightHandModel  # Added
                    VG.trueRightModelData = Clustered_data[index]  # Added
                    Data_dInteret = [VG.trueLeftModelData, VG.trueRightModelData]
                    Droite_dInteret = [VG.trueLeftHandModel, VG.trueRightHandModel]
                    VG.lineHistory.append(
                        [Droite_dInteret[0][0], Droite_dInteret[0][1], Droite_dInteret[1][0], Droite_dInteret[1][1]])
                    VG.counterPermanence = 0

                elif (abs(VG.trueRightHandModel[0] - rightHandModel[0]) <= 0.9) and (
                    abs(abs(rightHandModel[1] - VG.trueRightHandModel[1]) / VG.trueRightHandModel[1]) <= 0.2):
                    ##print("I've entered in (abs(VG.trueRightHandModel[0] - rightHandModel[0]) <= 0.5)")
                    VG.trueRightHandModel = rightHandModel  # Added
                    VG.trueRightModelData = Clustered_data[index]  # Added
                    Data_dInteret = [VG.trueRightModelData]
                    Droite_dInteret = [VG.trueRightHandModel]
                    VG.lineHistory.append([None, None, Droite_dInteret[0][0], Droite_dInteret[0][1]])
                    VG.trueLeftHandModel = []
                    VG.trueLeftModelData = []
                    VG.counterPermanence = 0

            elif not flagFoundModelsRight and VG.trueRightHandModel and not VG.trueLeftHandModel:
                ##print("## I have a true model, but not two parallel ones on the new reading testing if the new model is parallel to the old one...  ##")
                if (abs(VG.trueRightHandModel[0] - rightHandModel[0]) <= 0.4) and (
                    abs(abs(rightHandModel[1] - VG.trueRightHandModel[1]) / VG.trueRightHandModel[1]) <= 0.2):
                    ##print("Yes, they are parallel and not so far from each other!")
                    VG.trueRightHandModel = rightHandModel  # Added
                    VG.trueRightModelData = Clustered_data[index]  # Added
                    Data_dInteret = [VG.trueRightModelData]
                    Droite_dInteret = [VG.trueRightHandModel]
                    VG.lineHistory.append([None, None, Droite_dInteret[0][0], Droite_dInteret[0][1]])
                    VG.counterPermanence = 0
                else:
                    ##print("NewModel is not parallel enough")
                    Data_dInteret = [VG.trueRightModelData]
                    Droite_dInteret = [VG.trueRightHandModel]
                    VG.lineHistory.append([None, None, Droite_dInteret[0][0], Droite_dInteret[0][1]])
                    VG.counterPermanence = VG.counterPermanence + 1

            elif VG.trueLeftHandModel and not flagFoundModelsRight and not VG.trueRightHandModel:
                Data_dInteret = [VG.trueLeftModelData]
                Droite_dInteret = [VG.trueLeftHandModel]
                VG.lineHistory.append([Droite_dInteret[0][0], Droite_dInteret[0][1], None, None])
                VG.counterPermanence = VG.counterPermanence + 1

            else:
                VG.counterPermanence = VG.counterPermanence + 1

        elif leLeftModels and not leRightModels:# and not VG.flagInitModels:#
            #&#print("Entered: LeftHandModel and (not leRightModels)")
            # print("The true models are:\nleft Hand = ", VG.trueLeftHandModel, "\nright Hand = ", VG.trueRightHandModel)
            flagFoundModelsLeft = False

            for k in range(len(leLeftModels)):
                # print("Executing the for...", k)
                if (abs(leftHandModel[0] - leLeftModels[k][0]) <= 0.1) and (leftHandModel != leLeftModels[k]):
                    # print("I have two parallel Models in leLeftModels, they are:", leftHandModel, "and\n", leLeftModels[k])
                    flagFoundModelsLeft = True

            if VG.flagInitModels and ((abs(leftHandModel[0] - VG.leftHandAverage) <= 0.15) or flagFoundModelsLeft):#flagFoundModelsLeft:
                ##print("On Initialization")
                if VG.averageDist and ((abs((VG.averageDist/2) + leftHandModel[1])/(VG.averageDist/2)) < 1):
                    #-#print("New model is close enough!")
                    VG.trueLeftHandModel = leftHandModel
                    VG.trueLeftModelData = Clustered_data[index2]
                    VG.trueRightHandModel = []
                    VG.trueRightModelData = []
                    Data_dInteret = [VG.trueLeftModelData]
                    Droite_dInteret = [VG.trueLeftHandModel]
                    VG.lineHistory.append([Droite_dInteret[0][0], Droite_dInteret[0][1], None, None])
                    VG.counterPermanence = 0
                    VG.flagInitModels = False
                elif VG.averageDist and not ((abs((VG.averageDist/2) + leftHandModel[1])/(VG.averageDist/2)) < 1):
                    #-#print("New model is really far")
                    #-#print("leftHandModel[1] = ",leftHandModel[1])
                    VG.flagInitModels = True

                else:
                    print("No mean yet, not initializing with single Left model.")
                    """VG.trueLeftHandModel = leftHandModel
                    VG.trueLeftModelData = Clustered_data[index2]
                    VG.trueRightHandModel = []
                    VG.trueRightModelData = []
                    Data_dInteret = [VG.trueLeftModelData]
                    Droite_dInteret = [VG.trueLeftHandModel]
                    VG.lineHistory.append([Droite_dInteret[0][0], Droite_dInteret[0][1], None, None])
                    VG.counterPermanence = 0"""
                    VG.flagInitModels = True

            elif flagFoundModelsLeft and VG.trueLeftHandModel:
                # Thinking about considering: if the two models that are parallel between
                # themselves are also parallel with the trueLeft, do this, else, there's
                # Need for further testing such as: are the two new models parallel with
                # the right one, if there is one
                # VG.trueLeftHandModel = leftHandModel
                # VG.trueLeftModelData = Clustered_data[index2]
                if VG.trueRightHandModel and (abs(VG.trueRightHandModel[0] - leftHandModel[0]) <= 0.2):
                    ##print("I entered the routine with the condition: VG.trueRightHandModel and (abs(VG.trueRightHandModel[0] - leftHandModel[0]) <= 0.1)")
                    # if (abs(VG.trueRightHandModel[0] - VG.trueLeftHandModel[0]) <= 0.1):
                    VG.trueLeftHandModel = leftHandModel  # Added
                    VG.trueLeftModelData = Clustered_data[index2]  # Added
                    Data_dInteret = [VG.trueLeftModelData, VG.trueRightModelData]
                    Droite_dInteret = [VG.trueLeftHandModel, VG.trueRightHandModel]
                    VG.lineHistory.append(
                        [Droite_dInteret[0][0], Droite_dInteret[0][1], Droite_dInteret[1][0], Droite_dInteret[1][1]])
                    VG.counterPermanence = 0

                elif (abs(VG.trueLeftHandModel[0] - leftHandModel[0]) <= 0.9) and (
                    abs(abs(leftHandModel[1] - VG.trueLeftHandModel[1]) / VG.trueLeftHandModel[1]) <= 0.2):
                    # The exception for level 2 happened here.
                    # print("I've entered in (abs(VG.trueLeftHandModel[0] - leftHandModel[0]) <= 0.5)")
                    # print("The true models are:\nleft Hand = ", VG.trueLeftHandModel, "\nright Hand = ",VG.trueRightHandModel)
                    VG.trueLeftHandModel = leftHandModel  # Added
                    VG.trueLeftModelData = Clustered_data[index2]  # Added
                    Data_dInteret = [VG.trueLeftModelData]
                    Droite_dInteret = [VG.trueLeftHandModel]
                    VG.lineHistory.append([Droite_dInteret[0][0], Droite_dInteret[0][1], None, None])
                    VG.trueRightHandModel = []
                    VG.trueRightModelData = []
                    VG.counterPermanence = 0

            elif not flagFoundModelsLeft and VG.trueLeftHandModel and not VG.trueRightHandModel:
                ##print("## I have a true model, but not two parallel ones on the new reading testing if the new model is parallel to the old one...  ##")
                if (abs(VG.trueLeftHandModel[0] - leftHandModel[0]) <= 0.4) and (
                    abs(abs(leftHandModel[1] - VG.trueLeftHandModel[1]) / VG.trueLeftHandModel[1]) <= 0.2):
                    # print("Yes, they are parallel and close enough to eachother!")
                    VG.trueLeftHandModel = leftHandModel  # Added
                    VG.trueLeftModelData = Clustered_data[index2]  # Added
                    Data_dInteret = [VG.trueLeftModelData]
                    Droite_dInteret = [VG.trueLeftHandModel]
                    VG.lineHistory.append([Droite_dInteret[0][0], Droite_dInteret[0][1], None, None])
                    VG.counterPermanence = 0
                else:
                    # print("NewModel is not parallel enough")
                    Data_dInteret = [VG.trueLeftModelData]
                    Droite_dInteret = [VG.trueLeftHandModel]
                    VG.lineHistory.append([Droite_dInteret[0][0], Droite_dInteret[0][1], None, None])
                    VG.counterPermanence = VG.counterPermanence + 1

            elif not flagFoundModelsLeft and VG.trueRightHandModel and not VG.trueLeftHandModel:
                Data_dInteret = [VG.trueRightModelData]
                Droite_dInteret = [VG.trueRightHandModel]
                VG.lineHistory.append([None, None, Droite_dInteret[0][0], Droite_dInteret[0][1]])
                VG.counterPermanence = VG.counterPermanence + 1

            else:
                VG.counterPermanence = VG.counterPermanence + 1

        # print("The true models are:\nleft Hand = ",VG.trueLeftHandModel,"\nright Hand = ",VG.trueRightHandModel)
        # print("The Models that arrived for left and right hand were:\nleft Hand = ",leftHandModel,"\nright Hand = ",rightHandModel)
        # print("Data_dInteret=", Data_dInteret, "\nDroite_dInteret =", Droite_dInteret)

        if not (Droite_dInteret and Data_dInteret): # (Droite_dInteret and Data_dInteret)
        #-#print("Jumping the writing...")
        #-#else:
            if VG.trueLeftModelData and VG.trueRightModelData:
                Data_dInteret = [VG.trueLeftModelData, VG.trueRightModelData]
                Droite_dInteret = [VG.trueLeftHandModel, VG.trueRightHandModel]
                VG.lineHistory.append([Droite_dInteret[0][0], Droite_dInteret[0][1], Droite_dInteret[1][0], Droite_dInteret[1][1]])
                TypeRang = "Double"
            elif VG.trueLeftModelData and not VG.trueRightModelData:
                Data_dInteret = [VG.trueLeftModelData]
                Droite_dInteret = [VG.trueLeftHandModel]
                VG.lineHistory.append([Droite_dInteret[0][0], Droite_dInteret[0][1], None, None])
                TypeRang = "Gauche"
            elif VG.trueRightModelData and not VG.trueLeftModelData:
                Data_dInteret = [VG.trueRightModelData]
                Droite_dInteret = [VG.trueRightHandModel]
                VG.lineHistory.append([None, None, Droite_dInteret[0][0], Droite_dInteret[0][1]])
                TypeRang = "Droite"

        if len(Data_dInteret) == 2:
            TypeRang = "Double"
            # print("Coef b doite : " + str(Droite_dInteret[0][1]) + "\tCoef b gauche : " + str(Droite_dInteret[1][1]) + "\tDistance inter-rang : " + str(abs(Droite_dInteret[0][1]) + abs(Droite_dInteret[1][1])))
        elif len(Data_dInteret) == 1:
            if Droite_dInteret[0][1] > 0:
                TypeRang = "Droite"
                # print("Ecartement instantané : " + str(Droite_dInteret[0][1]))
            else:
                TypeRang = "Gauche"
                # print("Ecartement instantané : " + str(Droite_dInteret[0][1]))
        else:
            TypeRang = "Null"

        ##print("Data_dInteret = ",Data_dInteret,"\nDroite_dInteret = ",Droite_dInteret)

        self.preControlFunction()

        #-#print("The averages for the coefficients are:\nrightHandAverage =", VG.rightHandAverage,"\nleftHandAverage =", VG.leftHandAverage,"\nrightHandDist =", VG.rightHandDist,"\nleftHandAverage =", VG.leftHandDist,"\nAverage distance between rows =",VG.averageDist,"\nVG.targetDistSum =",VG.targetDistSum,"\nVG.distToTarget =",VG.distToTarget)

        return Data_dInteret, Droite_dInteret, TypeRang
    # -----------------------------------------------------------------------------------------------
    def filtrage_droitesFlex2(self, Clustered_data, Clustered_data_lines):
        # The same as filtrage_droitesFlex, but even more flexible in it's filtering, the difference from this to the
        # latter is that this one is activated only when the robot is falling back to face the new rows, this function
        # is the place to work at if we want to base that recoil's ending in finding the new rows to follow, so far,
        # this is not done once the fall back procedure is based on the Theta angle.
        min1B = float('-inf')
        min2B = float('inf')
        rightHandModel = []
        leftHandModel = []
        leLeftModels = []
        leRightModels = []
        Droite_dInteret = []
        Data_dInteret = []
        # print("Clustered_data_lines = ",Clustered_data_lines)

        for k in range(len(Clustered_data_lines)):
            # print("Clustered_data[k][2] = ",Clustered_data_lines[k][1])
            if (Clustered_data_lines[k][1] < 0):
                leLeftModels.append(Clustered_data_lines[k])
            if (Clustered_data_lines[k][1] > 0):
                leRightModels.append(Clustered_data_lines[k])

            if (Clustered_data_lines[k][1] > min1B) and (Clustered_data_lines[k][1] < 0):
                min1B = Clustered_data_lines[k][1]
                leftHandModel = Clustered_data_lines[k]
                # leLeftModels.append(Clustered_data_lines[k])
                index2 = k
            if (Clustered_data_lines[k][1] < min2B) and (Clustered_data_lines[k][1] > 0):
                min2B = Clustered_data_lines[k][1]
                rightHandModel = Clustered_data_lines[k]
                # leRightModels.append(Clustered_data_lines[k])
                index = k

        if VG.counterPermanence > 3:
            # print("The models that arrived were:\nleftHandModel =",leftHandModel,"\nrightHandModel =",rightHandModel)
            # print("we had\nTrueLeftHandModel =",VG.trueLeftHandModel,"\nTrueRightHandModel =",VG.trueRightHandModel)
            VG.flagInitModels = True
            VG.trueRightHandModel = []
            VG.trueLeftHandModel = []
            VG.trueRightModelData = []
            VG.trueLeftModelData = []
            VG.counterPermanence = 0
            #-#print("VG.trueRightHandModel =", VG.trueRightHandModel, "\nVG.trueLeftHandModel =", VG.trueLeftHandModel)
            #-#print("VG.lineHistory =\n", VG.lineHistory)
            #-#print("The averages for the coefficients are:\nrightHandAverage =", VG.rightHandAverage,"\nleftHandAverage =", VG.leftHandAverage)

        if VG.flagInitModels and rightHandModel and leftHandModel:
            #&#print("On Initialization both models...  ADDED SAFETY FOR INITIALIZATION ONLY WITH PERFECT MODEL")
            if rightHandModel and leftHandModel and (abs(rightHandModel[0] - leftHandModel[0]) <= 0.5) and not ((VG.rightHandAverage or VG.leftHandAverage) and (VG.averageDistToLeft or VG.averageDistToRight)) and not VG.averageDist:
                VG.trueRightHandModel = rightHandModel
                VG.trueLeftHandModel = leftHandModel
                VG.trueRightModelData = Clustered_data[index]
                VG.trueLeftModelData = Clustered_data[index2]
                VG.counterPermanence = 0
                VG.flagInitModels = False
            else:
                #-#print("initializing with average values already set")
                dist1 = abs(leftHandModel[1] - rightHandModel[1])
                if (abs(leftHandModel[0] - rightHandModel[0]) <= 0.2) and (abs(leftHandModel[0] - VG.leftHandAverage) <= 0.5) and (abs(rightHandModel[0] - VG.rightHandAverage) <= 0.5) and (VG.averageDist and (abs((VG.averageDist - dist1)/VG.averageDist) < 0.5)):  # and ((abs(dist1 - dist2)/dist2) < 0.7): #and (abs(VG.trueLeftHandModel[0] - leftHandModel[0]) <= 0.5) and (abs(VG.trueRightHandModel[0] - rightHandModel[0]) <= 0.5): # : # and VG.flagParallels:
                    # print("The new models are parallel, change will happen...")
                    VG.trueLeftHandModel = leftHandModel
                    VG.trueLeftModelData = Clustered_data[index2]
                    VG.trueRightHandModel = rightHandModel
                    VG.trueRightModelData = Clustered_data[index]
                    VG.counterPermanence = 0
                    VG.flagInitModels = False
        # Mark

        elif rightHandModel and leftHandModel and not VG.flagInitModels:
            #&#print("Entered: rightHandModel and leftHandModel")  # the Conditions after ((abs(rightHandModel[1] - VG.trueRightHandModel[1])/rightHandModel[1]) < 0.3) were added after the video niveau3

            if VG.trueRightHandModel and VG.trueLeftHandModel:
                # print("I have true models for both sides!")
                dist1 = abs(leftHandModel[1] - rightHandModel[1])
                dist2 = abs(VG.trueLeftHandModel[1] - VG.trueRightHandModel[1])
                if (abs(leftHandModel[0] - rightHandModel[0]) <= 0.2) and ((abs(leftHandModel[0] - VG.leftHandAverage) <= 0.4) or (abs(VG.trueLeftHandModel[0] - leftHandModel[0]) <= 0.7)) and ((abs(rightHandModel[0] - VG.rightHandAverage) <= 0.4) or (abs(VG.trueRightHandModel[0] - rightHandModel[0]) <= 0.7)) and (abs((VG.averageDist - dist1)/VG.averageDist) < 0.5):  # and ((abs(dist1 - dist2)/dist2) < 0.7): #and (abs(VG.trueLeftHandModel[0] - leftHandModel[0]) <= 0.5) and (abs(VG.trueRightHandModel[0] - rightHandModel[0]) <= 0.5): # : # and VG.flagParallels:
                    # print("The new models are parallel, change will happen...")
                    VG.trueLeftHandModel = leftHandModel
                    VG.trueLeftModelData = Clustered_data[index2]
                    VG.trueRightHandModel = rightHandModel
                    VG.trueRightModelData = Clustered_data[index]
                    VG.counterPermanence = 0
                else:
                    ##print("Not enough parallel!!!!\nKeeping old models!!!!")
                    # print("models are:\nVG.trueRightHandModel =",VG.trueRightHandModel,"\nVG.trueLeftHandModel =",VG.trueLeftHandModel,"\nRightHandModel =",rightHandModel,"\nleftHandModel",leftHandModel)
                    VG.counterPermanence = VG.counterPermanence + 1
                    # Data_dInteret = [VG.trueLeftModelData, VG.trueRightModelData]
                    # Droite_dInteret = [VG.trueLeftHandModel, VG.trueRightHandModel]
            else:
                # print("I had only one side of trueModels!!")

                # print("The models are:\nleftHandModel = ",leftHandModel,"\nAnd rightHandModel =",rightHandModel)

                if VG.trueLeftHandModel and not VG.trueRightHandModel:
                    # print("I had a leftHandModel but not a right hand one")
                    dist1 = abs(leftHandModel[1] - rightHandModel[1])
                    if (abs(leftHandModel[0] - rightHandModel[0]) <= 0.2) and ((abs(VG.trueLeftHandModel[0] - leftHandModel[0]) <= 1.3) or (abs(VG.leftHandAverage - leftHandModel[0]) <= 0.7)) and (abs((VG.averageDist - dist1)/VG.averageDist) < 0.5):
                        # print("The condition (abs(leftHandModel[0] - rightHandModel[0]) <= 0.2) and (abs(VG.trueLeftHandModel - leftHandModel[0]) <= 0.8) was satisfied, changing models...")
                        VG.trueLeftHandModel = leftHandModel
                        VG.trueLeftModelData = Clustered_data[index2]
                        VG.trueRightHandModel = rightHandModel
                        VG.trueRightModelData = Clustered_data[index]
                        Data_dInteret = [VG.trueLeftModelData, VG.trueRightModelData]
                        Droite_dInteret = [VG.trueLeftHandModel, VG.trueRightHandModel]
                        VG.lineHistory.append([Droite_dInteret[0][0], Droite_dInteret[0][1], Droite_dInteret[1][0],Droite_dInteret[1][1]])
                        VG.counterPermanence = 0

                    else:
                        # print("The big condition wasn't satisfied, so I'm keeping the old model.")
                        Data_dInteret = [VG.trueLeftModelData]
                        Droite_dInteret = [VG.trueLeftHandModel]
                        VG.lineHistory.append([Droite_dInteret[0][0], Droite_dInteret[0][1], None, None])
                        VG.counterPermanence = VG.counterPermanence + 1

                elif VG.trueRightHandModel and not VG.trueLeftHandModel:
                    # print("I had a rightHandModel but not a left hand one")
                    dist1 = abs(leftHandModel[1] - rightHandModel[1])
                    if (abs(leftHandModel[0] - rightHandModel[0]) <= 0.2) and ((abs(VG.trueRightHandModel[0] - rightHandModel[0]) <= 1.3) or (abs(VG.rightHandAverage - rightHandModel[0]) <= 0.7)) and (abs((VG.averageDist - dist1)/VG.averageDist) < 0.5):
                        # print("The condition (abs(leftHandModel[0] - rightHandModel[0]) <= 0.2) and (abs(VG.trueRightHandModel[0] - rightHandModel[0]) <= 0.8)   was satisfied, changing models...")
                        VG.trueLeftHandModel = leftHandModel
                        VG.trueLeftModelData = Clustered_data[index2]
                        VG.trueRightHandModel = rightHandModel
                        VG.trueRightModelData = Clustered_data[index]
                        Data_dInteret = [VG.trueLeftModelData, VG.trueRightModelData]
                        Droite_dInteret = [VG.trueLeftHandModel, VG.trueRightHandModel]
                        VG.lineHistory.append([Droite_dInteret[0][0], Droite_dInteret[0][1], Droite_dInteret[1][0],
                                               Droite_dInteret[1][1]])
                        VG.counterPermanence = 0
                    else:
                        # print("The big condition2 wasn't satisfied, so I'm keeping the old model.")
                        Data_dInteret = [VG.trueRightModelData]
                        Droite_dInteret = [VG.trueRightHandModel]
                        VG.lineHistory.append([None, None, Droite_dInteret[0][0], Droite_dInteret[0][1]])
                        VG.counterPermanence = VG.counterPermanence + 1

        elif leRightModels and not leLeftModels and not VG.flagInitModels:#:#
            #&#print("Entered: rightHandModels and (not leLeftModels)")
            # print("The true models are:\nleft Hand = ", VG.trueLeftHandModel, "\nright Hand = ", VG.trueRightHandModel)

            flagFoundModelsRight = False

            for k in range(len(leRightModels)):
                # print("Executing the for...", k)
                if (abs(rightHandModel[0] - leRightModels[k][0]) <= 0.1) and (rightHandModel != leRightModels[k]):
                    # print("I have two parallel Models in leRightModels, they are:",rightHandModel,"and\n",leRightModels[k])
                    flagFoundModelsRight = True

            if VG.flagInitModels and ((abs(rightHandModel[0] - VG.rightHandAverage) <= 0.5) or flagFoundModelsRight):#flagFoundModelsRight:
                ##print("On Initialization")
                if VG.averageDist and ((abs((VG.averageDist/2) - rightHandModel[1])/(VG.averageDist/2)) < 1):
                    #-#print("New model is close enough!")
                    VG.trueRightHandModel = rightHandModel
                    VG.trueRightModelData = Clustered_data[index]
                    VG.trueLeftHandModel = []
                    VG.trueLeftModelData = []
                    Data_dInteret = [VG.trueRightModelData]
                    Droite_dInteret = [VG.trueRightHandModel]
                    VG.lineHistory.append([None, None, Droite_dInteret[0][0], Droite_dInteret[0][1]])
                    VG.counterPermanence = 0
                    VG.flagInitModels = False
                elif VG.averageDist and not ((abs((VG.averageDist/2) - rightHandModel[1])/(VG.averageDist/2)) < 1):
                    #-#print("2New model is really far")
                    #-#print("rightHandModel[1] = ",rightHandModel[1])
                    #-#print("(abs((VG.averageDist/2) - rightHandModel[1]/(VG.averageDist/2)) = ",(abs((VG.averageDist/2) - rightHandModel[1])/(VG.averageDist/2)))
                    VG.flagInitModels = True
                else:
                    VG.trueRightHandModel = rightHandModel
                    VG.trueRightModelData = Clustered_data[index]
                    VG.trueLeftHandModel = []
                    VG.trueLeftModelData = []
                    Data_dInteret = [VG.trueRightModelData]
                    Droite_dInteret = [VG.trueRightHandModel]
                    VG.lineHistory.append([None, None, Droite_dInteret[0][0], Droite_dInteret[0][1]])
                    VG.counterPermanence = 0
                    VG.flagInitModels = False

            elif flagFoundModelsRight and VG.trueRightHandModel:
                if VG.trueLeftHandModel and (abs(VG.trueLeftHandModel[0] - rightHandModel[0]) <= 0.2):
                    VG.trueRightHandModel = rightHandModel  # Added
                    VG.trueRightModelData = Clustered_data[index]  # Added
                    Data_dInteret = [VG.trueLeftModelData, VG.trueRightModelData]
                    Droite_dInteret = [VG.trueLeftHandModel, VG.trueRightHandModel]
                    VG.lineHistory.append(
                        [Droite_dInteret[0][0], Droite_dInteret[0][1], Droite_dInteret[1][0], Droite_dInteret[1][1]])
                    VG.counterPermanence = 0

                elif (abs(VG.trueRightHandModel[0] - rightHandModel[0]) <= 0.9) and (
                    abs(abs(rightHandModel[1] - VG.trueRightHandModel[1]) / VG.trueRightHandModel[1]) <= 0.2):
                    ##print("I've entered in (abs(VG.trueRightHandModel[0] - rightHandModel[0]) <= 0.5)")
                    VG.trueRightHandModel = rightHandModel  # Added
                    VG.trueRightModelData = Clustered_data[index]  # Added
                    Data_dInteret = [VG.trueRightModelData]
                    Droite_dInteret = [VG.trueRightHandModel]
                    VG.lineHistory.append([None, None, Droite_dInteret[0][0], Droite_dInteret[0][1]])
                    VG.trueLeftHandModel = []
                    VG.trueLeftModelData = []
                    VG.counterPermanence = 0

            elif not flagFoundModelsRight and VG.trueRightHandModel and not VG.trueLeftHandModel:
                ##print("## I have a true model, but not two parallel ones on the new reading testing if the new model is parallel to the old one...  ##")
                if (abs(VG.trueRightHandModel[0] - rightHandModel[0]) <= 0.4) and (
                    abs(abs(rightHandModel[1] - VG.trueRightHandModel[1]) / VG.trueRightHandModel[1]) <= 0.2):
                    ##print("Yes, they are parallel and not so far from each other!")
                    VG.trueRightHandModel = rightHandModel  # Added
                    VG.trueRightModelData = Clustered_data[index]  # Added
                    Data_dInteret = [VG.trueRightModelData]
                    Droite_dInteret = [VG.trueRightHandModel]
                    VG.lineHistory.append([None, None, Droite_dInteret[0][0], Droite_dInteret[0][1]])
                    VG.counterPermanence = 0
                else:
                    ##print("NewModel is not parallel enough")
                    Data_dInteret = [VG.trueRightModelData]
                    Droite_dInteret = [VG.trueRightHandModel]
                    VG.lineHistory.append([None, None, Droite_dInteret[0][0], Droite_dInteret[0][1]])
                    VG.counterPermanence = VG.counterPermanence + 1

            elif VG.trueLeftHandModel and not flagFoundModelsRight and not VG.trueRightHandModel:
                Data_dInteret = [VG.trueLeftModelData]
                Droite_dInteret = [VG.trueLeftHandModel]
                VG.lineHistory.append([Droite_dInteret[0][0], Droite_dInteret[0][1], None, None])
                VG.counterPermanence = VG.counterPermanence + 1

            else:
                VG.counterPermanence = VG.counterPermanence + 1

        elif leLeftModels and not leRightModels and not VG.flagInitModels:#:#
            #&#print("Entered: LeftHandModel and (not leRightModels)")
            # print("The true models are:\nleft Hand = ", VG.trueLeftHandModel, "\nright Hand = ", VG.trueRightHandModel)
            flagFoundModelsLeft = False

            for k in range(len(leLeftModels)):
                # print("Executing the for...", k)
                if (abs(leftHandModel[0] - leLeftModels[k][0]) <= 0.1) and (leftHandModel != leLeftModels[k]):
                    # print("I have two parallel Models in leLeftModels, they are:", leftHandModel, "and\n", leLeftModels[k])
                    flagFoundModelsLeft = True

            if VG.flagInitModels and ((abs(leftHandModel[0] - VG.leftHandAverage) <= 0.5) or flagFoundModelsLeft):#flagFoundModelsLeft:
                ##print("On Initialization")
                if VG.averageDist and ((abs((VG.averageDist/2) + leftHandModel[1])/(VG.averageDist/2)) < 1):
                    #-#print("New model is close enough!")
                    VG.trueLeftHandModel = leftHandModel
                    VG.trueLeftModelData = Clustered_data[index2]
                    VG.trueRightHandModel = []
                    VG.trueRightModelData = []
                    Data_dInteret = [VG.trueLeftModelData]
                    Droite_dInteret = [VG.trueLeftHandModel]
                    VG.lineHistory.append([Droite_dInteret[0][0], Droite_dInteret[0][1], None, None])
                    VG.counterPermanence = 0
                    VG.flagInitModels = False
                elif VG.averageDist and not ((abs((VG.averageDist/2) + leftHandModel[1])/(VG.averageDist/2)) < 1):
                    #-#print("New model is really far")
                    #-#print("leftHandModel[1] = ",leftHandModel[1])
                    VG.flagInitModels = True

                else:
                    VG.trueLeftHandModel = leftHandModel
                    VG.trueLeftModelData = Clustered_data[index2]
                    VG.trueRightHandModel = []
                    VG.trueRightModelData = []
                    Data_dInteret = [VG.trueLeftModelData]
                    Droite_dInteret = [VG.trueLeftHandModel]
                    VG.lineHistory.append([Droite_dInteret[0][0], Droite_dInteret[0][1], None, None])
                    VG.counterPermanence = 0
                    VG.flagInitModels = False

            elif flagFoundModelsLeft and VG.trueLeftHandModel:
                # Thinking about considering: if the two models that are parallel between
                # themselves are also parallel with the trueLeft, do this, else, there's
                # Need for further testing such as: are the two new models parallel with
                # the right one, if there is one
                # VG.trueLeftHandModel = leftHandModel
                # VG.trueLeftModelData = Clustered_data[index2]
                if VG.trueRightHandModel and (abs(VG.trueRightHandModel[0] - leftHandModel[0]) <= 0.2):
                    ##print("I entered the routine with the condition: VG.trueRightHandModel and (abs(VG.trueRightHandModel[0] - leftHandModel[0]) <= 0.1)")
                    # if (abs(VG.trueRightHandModel[0] - VG.trueLeftHandModel[0]) <= 0.1):
                    VG.trueLeftHandModel = leftHandModel  # Added
                    VG.trueLeftModelData = Clustered_data[index2]  # Added
                    Data_dInteret = [VG.trueLeftModelData, VG.trueRightModelData]
                    Droite_dInteret = [VG.trueLeftHandModel, VG.trueRightHandModel]
                    VG.lineHistory.append(
                        [Droite_dInteret[0][0], Droite_dInteret[0][1], Droite_dInteret[1][0], Droite_dInteret[1][1]])
                    VG.counterPermanence = 0

                elif (abs(VG.trueLeftHandModel[0] - leftHandModel[0]) <= 0.9) and (
                    abs(abs(leftHandModel[1] - VG.trueLeftHandModel[1]) / VG.trueLeftHandModel[1]) <= 0.2):
                    # The exception for level 2 happened here.
                    # print("I've entered in (abs(VG.trueLeftHandModel[0] - leftHandModel[0]) <= 0.5)")
                    # print("The true models are:\nleft Hand = ", VG.trueLeftHandModel, "\nright Hand = ",VG.trueRightHandModel)
                    VG.trueLeftHandModel = leftHandModel  # Added
                    VG.trueLeftModelData = Clustered_data[index2]  # Added
                    Data_dInteret = [VG.trueLeftModelData]
                    Droite_dInteret = [VG.trueLeftHandModel]
                    VG.lineHistory.append([Droite_dInteret[0][0], Droite_dInteret[0][1], None, None])
                    VG.trueRightHandModel = []
                    VG.trueRightModelData = []
                    VG.counterPermanence = 0

            elif not flagFoundModelsLeft and VG.trueLeftHandModel and not VG.trueRightHandModel:
                ##print("## I have a true model, but not two parallel ones on the new reading testing if the new model is parallel to the old one...  ##")
                if (abs(VG.trueLeftHandModel[0] - leftHandModel[0]) <= 0.4) and (
                    abs(abs(leftHandModel[1] - VG.trueLeftHandModel[1]) / VG.trueLeftHandModel[1]) <= 0.2):
                    # print("Yes, they are parallel and close enough to eachother!")
                    VG.trueLeftHandModel = leftHandModel  # Added
                    VG.trueLeftModelData = Clustered_data[index2]  # Added
                    Data_dInteret = [VG.trueLeftModelData]
                    Droite_dInteret = [VG.trueLeftHandModel]
                    VG.lineHistory.append([Droite_dInteret[0][0], Droite_dInteret[0][1], None, None])
                    VG.counterPermanence = 0
                else:
                    # print("NewModel is not parallel enough")
                    Data_dInteret = [VG.trueLeftModelData]
                    Droite_dInteret = [VG.trueLeftHandModel]
                    VG.lineHistory.append([Droite_dInteret[0][0], Droite_dInteret[0][1], None, None])
                    VG.counterPermanence = VG.counterPermanence + 1

            elif not flagFoundModelsLeft and VG.trueRightHandModel and not VG.trueLeftHandModel:
                Data_dInteret = [VG.trueRightModelData]
                Droite_dInteret = [VG.trueRightHandModel]
                VG.lineHistory.append([None, None, Droite_dInteret[0][0], Droite_dInteret[0][1]])
                VG.counterPermanence = VG.counterPermanence + 1

            else:
                VG.counterPermanence = VG.counterPermanence + 1

        # print("The true models are:\nleft Hand = ",VG.trueLeftHandModel,"\nright Hand = ",VG.trueRightHandModel)
        # print("The Models that arrived for left and right hand were:\nleft Hand = ",leftHandModel,"\nright Hand = ",rightHandModel)
        # print("Data_dInteret=", Data_dInteret, "\nDroite_dInteret =", Droite_dInteret)

        if not (Droite_dInteret and Data_dInteret): # (Droite_dInteret and Data_dInteret)
        #-#print("Jumping the writing...")
        #-#else:
            if VG.trueLeftModelData and VG.trueRightModelData:
                Data_dInteret = [VG.trueLeftModelData, VG.trueRightModelData]
                Droite_dInteret = [VG.trueLeftHandModel, VG.trueRightHandModel]
                VG.lineHistory.append([Droite_dInteret[0][0], Droite_dInteret[0][1], Droite_dInteret[1][0], Droite_dInteret[1][1]])
                TypeRang = "Double"
            elif VG.trueLeftModelData and not VG.trueRightModelData:
                Data_dInteret = [VG.trueLeftModelData]
                Droite_dInteret = [VG.trueLeftHandModel]
                VG.lineHistory.append([Droite_dInteret[0][0], Droite_dInteret[0][1], None, None])
                TypeRang = "Gauche"
            elif VG.trueRightModelData and not VG.trueLeftModelData:
                Data_dInteret = [VG.trueRightModelData]
                Droite_dInteret = [VG.trueRightHandModel]
                VG.lineHistory.append([None, None, Droite_dInteret[0][0], Droite_dInteret[0][1]])
                TypeRang = "Droite"

        if len(Data_dInteret) == 2:
            TypeRang = "Double"
            # print("Coef b doite : " + str(Droite_dInteret[0][1]) + "\tCoef b gauche : " + str(Droite_dInteret[1][1]) + "\tDistance inter-rang : " + str(abs(Droite_dInteret[0][1]) + abs(Droite_dInteret[1][1])))
        elif len(Data_dInteret) == 1:
            if Droite_dInteret[0][1] > 0:
                TypeRang = "Droite"
                # print("Ecartement instantané : " + str(Droite_dInteret[0][1]))
            else:
                TypeRang = "Gauche"
                # print("Ecartement instantané : " + str(Droite_dInteret[0][1]))
        else:
            TypeRang = "Null"

        ##print("Data_dInteret = ",Data_dInteret,"\nDroite_dInteret = ",Droite_dInteret)

        #self.preControlFunction()

        """if VG.lineHistory:
            if VG.lineHistory[len(VG.lineHistory)-1][0] is not None:
                VG.leftHandSum = VG.leftHandSum + VG.lineHistory[len(VG.lineHistory)-1][0]
            if VG.lineHistory[len(VG.lineHistory)-1][2] is not None:
                VG.rightHandSum = VG.rightHandSum + VG.lineHistory[len(VG.lineHistory)-1][2]

            if VG.lineHistory[len(VG.lineHistory)-1][1] is not None:
                VG.leftHandDistSum = VG.leftHandDistSum + VG.lineHistory[len(VG.lineHistory)-1][1]
            if VG.lineHistory[len(VG.lineHistory)-1][3] is not None:
                VG.rightHandDistSum = VG.rightHandDistSum + VG.lineHistory[len(VG.lineHistory)-1][3]

            if (VG.lineHistory[len(VG.lineHistory)-1][3] is not None) and (VG.lineHistory[len(VG.lineHistory)-1][1] is not None):

                VG.distSum.append(abs(VG.lineHistory[len(VG.lineHistory)-1][3] - VG.lineHistory[len(VG.lineHistory)-1][1]))
                VG.targetDistSum.append(VG.lineHistory[len(VG.lineHistory)-1][3] + VG.lineHistory[len(VG.lineHistory)-1][1])

            elif (VG.lineHistory[len(VG.lineHistory)-1][3] is None) and (VG.lineHistory[len(VG.lineHistory)-1][1] is not None):
                print("Single ROW Left")
                print("SRowL Distance\n(VG.averageDist/2)-VG.lineHistory[len(VG.lineHistory)-1][1]=",(VG.averageDist/2)-VG.lineHistory[len(VG.lineHistory)-1][1])
                #VG.distSum = VG.distSum + (VG.lineHistory[len(VG.lineHistory)-1][1]+(VG.averageDist/2))
                #VG.targetDistSum.append()
                VG.distToTarget = (VG.lineHistory[len(VG.lineHistory)-1][1]+(VG.averageDist/2))
                VG.targetDistSum.append(VG.lineHistory[len(VG.lineHistory)-1][1]+(VG.averageDist/2))

            elif (VG.lineHistory[len(VG.lineHistory)-1][3] is not None) and (VG.lineHistory[len(VG.lineHistory)-1][1] is None):
                print("Single ROW Right")
                print("SRowR Distance\n(VG.averageDist/2)-VG.lineHistory[len(VG.lineHistory)-1][3]=",(VG.averageDist/2)-VG.lineHistory[len(VG.lineHistory)-1][3])
                #VG.distSum = VG.distSum + (VG.lineHistory[len(VG.lineHistory)-1][1]+(VG.averageDist/2))
                #VG.targetDistSum.append()
                VG.distToTarget = (VG.lineHistory[len(VG.lineHistory)-1][3]-(VG.averageDist/2))
                VG.targetDistSum.append(VG.lineHistory[len(VG.lineHistory)-1][3]-(VG.averageDist/2))

            VG.rightHandAverage = VG.rightHandSum / len(VG.lineHistory)
            VG.leftHandAverage = VG.leftHandSum / len(VG.lineHistory)
            VG.rightHandDist = VG.rightHandDistSum / len(VG.lineHistory)
            VG.leftHandDist = VG.leftHandDistSum / len(VG.lineHistory)

            #VG.targetDistAvg = VG.distSum / len(VG.lineHistory)

            VG.averageDist = sum(VG.distSum) / len(VG.distSum)"""

        #-#print("The averages for the coefficients are:\nrightHandAverage =", VG.rightHandAverage,"\nleftHandAverage =", VG.leftHandAverage,"\nrightHandDist =", VG.rightHandDist,"\nleftHandAverage =", VG.leftHandDist,"\nAverage distance between rows =",VG.averageDist,"\nVG.targetDistSum =",VG.targetDistSum,"\nVG.distToTarget =",VG.distToTarget)

        return Data_dInteret, Droite_dInteret, TypeRang
    # -----------------------------------------------------------------------------------------------
    def preControlFunction(self,):
        # Calculates the error to target (used then by the RFI filter) and several other parameters such as average
        # steepness for right and left models and their average distance (which allows us to know the average distance
        # between rows, a very important parameter). This function also considers the scenario in which the robot has a
        # too great relative angle to the rows, if that happens, there's the possibility that the robot may find itself
        # equidistant to the rows, although it is actually facing one of them, if so, a different way of calculating
        # the error is used and this one is based on the distance to intercept the closest model, and is multiplied by a
        # quite agressive factor, in order to speed up the correction of that scenario.

        if VG.lineHistory:
            if VG.lineHistory[len(VG.lineHistory)-1][0] is not None:
                VG.leftHandSum = VG.leftHandSum + VG.lineHistory[len(VG.lineHistory)-1][0]
            if VG.lineHistory[len(VG.lineHistory)-1][2] is not None:
                VG.rightHandSum = VG.rightHandSum + VG.lineHistory[len(VG.lineHistory)-1][2]

            if VG.lineHistory[len(VG.lineHistory)-1][1] is not None:
                VG.leftHandDistSum = VG.leftHandDistSum + VG.lineHistory[len(VG.lineHistory)-1][1]
            if VG.lineHistory[len(VG.lineHistory)-1][3] is not None:
                VG.rightHandDistSum = VG.rightHandDistSum + VG.lineHistory[len(VG.lineHistory)-1][3]

            if (VG.lineHistory[len(VG.lineHistory)-1][3] is not None) and (VG.lineHistory[len(VG.lineHistory)-1][1] is not None):
                #++# As it was in 02/05/17, Modifications towards adding penalties for when the lines are too steep in order to increase the control:
                # Original version:
                """VG.distSum.append(abs(VG.lineHistory[len(VG.lineHistory)-1][3] - VG.lineHistory[len(VG.lineHistory)-1][1]))
                VG.targetDistSum.append(VG.lineHistory[len(VG.lineHistory)-1][3] + VG.lineHistory[len(VG.lineHistory)-1][1])
                #VG.speedMax = 100
                #++=++## Modification that didn't work:
                if (VG.lineHistory[len(VG.lineHistory)-1][2] > 0.5):
                    VG.distSum.append(abs(VG.lineHistory[len(VG.lineHistory) - 1][3] - VG.lineHistory[len(VG.lineHistory) - 1][1]))
                    VG.targetDistSum.append(VG.lineHistory[len(VG.lineHistory) - 1][3] + (VG.lineHistory[len(VG.lineHistory) - 1][1] * 2))
                elif VG.lineHistory[len(VG.lineHistory)-1][2] < -0.5:
                    VG.distSum.append(abs(VG.lineHistory[len(VG.lineHistory) - 1][3] - VG.lineHistory[len(VG.lineHistory) - 1][1]))
                    VG.targetDistSum.append((VG.lineHistory[len(VG.lineHistory) - 1][3] * 2) + VG.lineHistory[len(VG.lineHistory) - 1][1])
                else:
                    VG.distSum.append(abs(VG.lineHistory[len(VG.lineHistory) - 1][3] - VG.lineHistory[len(VG.lineHistory) - 1][1]))
                    VG.targetDistSum.append(VG.lineHistory[len(VG.lineHistory) - 1][3] + VG.lineHistory[len(VG.lineHistory) - 1][1])"""

                #@@# Modification considering that models that are too steep get a different treatment (12/05/17).
                currentError = (VG.lineHistory[len(VG.lineHistory) - 1][3] + VG.lineHistory[len(VG.lineHistory) - 1][1])

                if ((VG.lineHistory[len(VG.lineHistory)-1][2] > 0.16) or (VG.lineHistory[len(VG.lineHistory)-1][0] > 0.16)) and (abs(currentError) < self._erThresh):
                    print("Steep model 1")
                    #VG.speedMax = self._initSpeedMax/2
                    if (VG.lineHistory[len(VG.lineHistory)-1][2] > 0.16) and VG.averageDist:
                        print("Sol 1")
                        VG.distSum.append(abs(VG.lineHistory[len(VG.lineHistory) - 1][3] - VG.lineHistory[len(VG.lineHistory) - 1][1]))
                        VG.targetDistSum.append(((- (VG.lineHistory[len(VG.lineHistory)-1][0])) / (VG.lineHistory[len(VG.lineHistory)-1][1])) * (350 * VG.averageDist))

                    elif (VG.lineHistory[len(VG.lineHistory)-1][2] > 0.16) and not VG.averageDist:
                        print("Sol 2")
                        VG.distSum.append(abs(VG.lineHistory[len(VG.lineHistory) - 1][3] - VG.lineHistory[len(VG.lineHistory) - 1][1]))
                        VG.targetDistSum.append((- (VG.lineHistory[len(VG.lineHistory)-1][0])) * 350)

                    elif (VG.lineHistory[len(VG.lineHistory)-1][0] > 0.16) and VG.averageDist:
                        print("Sol 3")
                        VG.distSum.append(abs(VG.lineHistory[len(VG.lineHistory) - 1][3] - VG.lineHistory[len(VG.lineHistory) - 1][1]))
                        VG.targetDistSum.append(((- (VG.lineHistory[len(VG.lineHistory)-1][0])) / (VG.lineHistory[len(VG.lineHistory)-1][1])) * (350 * VG.averageDist))

                    elif (VG.lineHistory[len(VG.lineHistory)-1][0] > 0.16) and not VG.averageDist:
                        print("Sol 4")
                        VG.distSum.append(abs(VG.lineHistory[len(VG.lineHistory) - 1][3] - VG.lineHistory[len(VG.lineHistory) - 1][1]))
                        VG.targetDistSum.append((- (VG.lineHistory[len(VG.lineHistory)-1][0])) * 350)

                    else:
                        print("Not Taking any action")

                    print("Motors = [",VG.speedLeftMotor,",",VG.speedRightMotor,"]")

                elif (VG.lineHistory[len(VG.lineHistory)-1][2] < -0.16) or (VG.lineHistory[len(VG.lineHistory)-1][0] < -0.16) and (abs(currentError) < self._erThresh):
                    print("Steep model 2")
                    #VG.speedMax = self._initSpeedMax / 2
                    if (VG.lineHistory[len(VG.lineHistory)-1][0] < -0.16) and VG.averageDist:
                        print("Sol 1")
                        VG.distSum.append(abs(VG.lineHistory[len(VG.lineHistory) - 1][3] - VG.lineHistory[len(VG.lineHistory) - 1][1]))
                        """calculdelta = ((( -(VG.lineHistory[len(VG.lineHistory)-1][0])) / (VG.lineHistory[len(VG.lineHistory)-1][1])) * (350 * VG.averageDist))
                        print("calculdelta = ",calculdelta)
                        #print("VG.averageDist = ",VG.averageDist)

                        kaa = []
                        if len(VG.targetDistSum) > 10:
                            for i in range (10):
                                kaa.append(VG.targetDistSum[len(VG.targetDistSum)-(i+1)])
                            #print("Historic of errors at this point =",kaa)"""
                        VG.targetDistSum.append(((( -(VG.lineHistory[len(VG.lineHistory)-1][0])) / (VG.lineHistory[len(VG.lineHistory)-1][1])) * (350 * VG.averageDist)))

                    elif (VG.lineHistory[len(VG.lineHistory)-1][0] < -0.16) and not VG.averageDist:
                        print("Sol 2")
                        VG.distSum.append(abs(VG.lineHistory[len(VG.lineHistory) - 1][3] - VG.lineHistory[len(VG.lineHistory) - 1][1]))
                        VG.targetDistSum.append((- (VG.lineHistory[len(VG.lineHistory)-1][0])) * 350)

                    elif (VG.lineHistory[len(VG.lineHistory)-1][2] < -0.16) and not VG.averageDist:
                        print("Sol 3")
                        VG.distSum.append(abs(VG.lineHistory[len(VG.lineHistory) - 1][3] - VG.lineHistory[len(VG.lineHistory) - 1][1]))
                        VG.targetDistSum.append(-((( -(VG.lineHistory[len(VG.lineHistory)-1][0])) / (VG.lineHistory[len(VG.lineHistory)-1][1])) * (350 * VG.averageDist)))

                    elif (VG.lineHistory[len(VG.lineHistory) - 1][2] < -0.16) and not VG.averageDist:
                        print("Sol 4")
                        VG.distSum.append(abs(VG.lineHistory[len(VG.lineHistory) - 1][3] - VG.lineHistory[len(VG.lineHistory) - 1][1]))
                        VG.targetDistSum.append((- (VG.lineHistory[len(VG.lineHistory)-1][0])) * 350)

                    else:
                        print("Not Taking any action")

                    print("Motors = [", VG.speedLeftMotor, ",", VG.speedRightMotor, "]")

                # End of modification:
                else:
                    VG.speedMax = self._initSpeedMax
                    VG.distSum.append(abs(VG.lineHistory[len(VG.lineHistory) - 1][3] - VG.lineHistory[len(VG.lineHistory) - 1][1]))
                    VG.targetDistSum.append((VG.lineHistory[len(VG.lineHistory) - 1][3] + VG.lineHistory[len(VG.lineHistory) - 1][1]) / 1.6)

            elif (VG.lineHistory[len(VG.lineHistory)-1][3] is None) and (VG.lineHistory[len(VG.lineHistory)-1][1] is not None):
                #&#print("Single ROW Left")
                #-#print("SRowL Distance\n(VG.averageDist/2)-VG.lineHistory[len(VG.lineHistory)-1][1]=",(VG.averageDist/2)-VG.lineHistory[len(VG.lineHistory)-1][1])
                #VG.distSum = VG.distSum + (VG.lineHistory[len(VG.lineHistory)-1][1]+(VG.averageDist/2))
                #VG.targetDistSum.append()
                if VG.averageDist:
                    VG.distToTarget = (VG.lineHistory[len(VG.lineHistory)-1][1]+(VG.averageDist/2))
                    VG.targetDistSum.append(VG.lineHistory[len(VG.lineHistory)-1][1]+(VG.averageDist/2))
                else:
                    VG.distToLeft.append(VG.lineHistory[len(VG.lineHistory)-1][1])
                    VG.targetDistSum.append(VG.lineHistory[len(VG.lineHistory) - 1][1] + VG.defaultDistance)
                    VG.distSum.append(2 * abs(VG.lineHistory[len(VG.lineHistory) - 1][1]))
                    #-#print("VG.distToLeft = ",VG.distToLeft)


            elif (VG.lineHistory[len(VG.lineHistory)-1][3] is not None) and (VG.lineHistory[len(VG.lineHistory)-1][1] is None):
                #&#print("Single ROW Right")
                #-#print("SRowR Distance\n(VG.averageDist/2)-VG.lineHistory[len(VG.lineHistory)-1][3]=",(VG.averageDist/2)-VG.lineHistory[len(VG.lineHistory)-1][3])
                #VG.distSum = VG.distSum + (VG.lineHistory[len(VG.lineHistory)-1][1]+(VG.averageDist/2))
                #VG.targetDistSum.append()
                if VG.averageDist:
                    VG.distToTarget = (VG.lineHistory[len(VG.lineHistory)-1][3]-(VG.averageDist/2))
                    VG.targetDistSum.append(-VG.lineHistory[len(VG.lineHistory)-1][3]+(VG.averageDist/2))
                else:
                    VG.distToRight.append(VG.lineHistory[len(VG.lineHistory)-1][3])
                    VG.targetDistSum.append(-VG.lineHistory[len(VG.lineHistory) - 1][3] + VG.defaultDistance)
                    VG.distSum.append(2*abs(VG.lineHistory[len(VG.lineHistory)-1][3]))
                    #-#print("VG.distToRight = ",VG.distToRight)

            VG.rightHandAverage = VG.rightHandSum / len(VG.lineHistory)
            VG.leftHandAverage = VG.leftHandSum / len(VG.lineHistory)
            VG.rightHandDist = VG.rightHandDistSum / len(VG.lineHistory)
            VG.leftHandDist = VG.leftHandDistSum / len(VG.lineHistory)

            #VG.targetDistAvg = VG.distSum / len(VG.lineHistory)
            if len(VG.distSum):
                VG.averageDist = sum(VG.distSum) / len(VG.distSum)
            if VG.distToRight:
                VG.averageDistToRight = sum(VG.distToRight) / len(VG.distToRight)
            if VG.distToLeft:
                VG.averageDistToLeft = sum(VG.distToLeft) / len(VG.distToLeft)

            return
    # -----------------------------------------------------------------------------------------------
    def filtrage_droites6(self, Clustered_data, Clustered_data_lines):
        # Filtering the straight lines for the find_lines2, as this method is called for changing rows, this filter is
        # optimized for finding models that are only at one "side" of the robot, once the robot is now looking foward
        # and the models that will be proposed will be only at one side of the carthesian axis.
        min1B = float('-inf')
        min2B = float('inf')
        rightHandModel = []
        leftHandModel = []
        leLeftModels = []
        leRightModels = []
        Droite_dInteret = []
        Data_dInteret = []
        leLeftData = []
        leRightData = []

        # print("Clustered_data_lines = ",Clustered_data_lines)
        for k in range(len(Clustered_data_lines)):
            # print("Clustered_data[k][2] = ",Clustered_data_lines[k][1])
            if (Clustered_data_lines[k][1] < 0):
                leLeftModels.append(Clustered_data_lines[k])
                leLeftData.append(Clustered_data[k])
            if (Clustered_data_lines[k][1] > 0):
                leRightModels.append(Clustered_data_lines[k])
                leRightData.append(Clustered_data[k])

            if (Clustered_data_lines[k][1] > min1B) and (Clustered_data_lines[k][1] < 0):
                min1B = Clustered_data_lines[k][1]
                leftHandModel = Clustered_data_lines[k]
                # leLeftModels.append(Clustered_data_lines[k])
                index2 = k
            if (Clustered_data_lines[k][1] < min2B) and (Clustered_data_lines[k][1] > 0):
                min2B = Clustered_data_lines[k][1]
                rightHandModel = Clustered_data_lines[k]
                # leRightModels.append(Clustered_data_lines[k])
                index = k

        if VG.counterPermanence2 > 6:
            # print("The models that arrived were:\nleftHandModel =",leftHandModel,"\nrightHandModel =",rightHandModel)
            # print("we had\nTrueLeftHandModel =",VG.trueLeftHandModel,"\nTrueRightHandModel =",VG.trueRightHandModel)
            VG.flagInitModels2 = True
            VG.trueRightHandModelPearl2 = []
            VG.trueSecondRightModel = []
            VG.trueRightModelDataPearl2 = []
            VG.trueSecondRightData = []
            VG.counterPermanence2 = 0

        # print("leLeftModels = ",leLeftModels)
        # print("leRightModels = ",leRightModels)

        if VG.flagInitModels2 and VG.flagInitModels3:
            if rightHandModel and leftHandModel and (abs(rightHandModel[0] - leftHandModel[0]) <= 0.1):
                print("WRONG: Models on both sides")

        if rightHandModel and leftHandModel and not VG.flagInitModels2 and not VG.flagInitModels3:
            a = 2
            VG.counterPermanence2 = VG.counterPermanence2 + 1
            #&#print("Entered: rightHandModel and leftHandModel WRONG!!")  # the Conditions after ((abs(rightHandModel[1] - VG.trueRightHandModelPearl2[1])/rightHandModel[1]) < 0.3) were added after the video niveau3

        elif leRightModels and not leLeftModels:# and (VG.directionTurn == "Left"):
            #&#print("Entered: rightHandModels and (not leLeftModels)")
            #-#print("The true models are:\nleft Hand = ", VG.trueLeftHandModelPearl2, "\nright Hand = ", VG.trueRightHandModelPearl2)
            flagFoundModelsRight = False
            VG.trueLeftModelDataPearl2 = []
            VG.trueLeftHandModelPearl2 = []
            rightSecondModel = []
            rightSecondData = []

            #-#print("leRight ModelsBefore sort method",leRightModels)
            leRightModels.sort(key=lambda x: x[1])
            #-#print("leRight Models After sort method", leRightModels)

            rightHandModel = leRightModels[0]
            #rightHandData

            for k in range(len(leRightModels)):
                # print("Executing the for...", k)
                if (abs(rightHandModel[0] - leRightModels[k][0]) <= 0.3) and (rightHandModel != leRightModels[k]):
                    #-#print("I have two parallel Models in leRightModels, they are:",rightHandModel,"and\n",leRightModels[k])
                    rightSecondModel = leRightModels[k]
                    rightSecondData = leRightData[k]
                    flagFoundModelsRight = True

            if VG.flagInitModels2 and flagFoundModelsRight:
                #&#print("On Initialization")
                VG.trueRightHandModelPearl2 = rightHandModel
                VG.trueRightModelDataPearl2 = Clustered_data[index]
                VG.trueSecondRightModel = rightSecondModel
                VG.trueSecondRightData = rightSecondData
                Data_dInteret = [VG.trueRightModelDataPearl2,VG.trueSecondRightData]
                Droite_dInteret = [VG.trueRightHandModelPearl2,VG.trueSecondRightModel]
                VG.minimalBCoefficient = Droite_dInteret[0][1]
                if VG.flagWarning:
                    VG.masterACoefficient = float('inf')
                else:
                    VG.masterACoefficient = Droite_dInteret[0][0]
                VG.counterPermanence2 = 0
                VG.flagInitModels2 = False

            elif VG.flagInitModels2 and not flagFoundModelsRight:
                #&#print("Initializing with only one model...")
                if abs(rightHandModel[0]) < 0.3:
                    VG.trueRightHandModelPearl2 = rightHandModel
                    VG.trueRightModelDataPearl2 = Clustered_data[index]
                    Data_dInteret = [VG.trueRightModelDataPearl2]
                    Droite_dInteret = [VG.trueRightHandModelPearl2]
                    VG.minimalBCoefficient = Droite_dInteret[0][1]
                    #-#if rightHandModel[1] < VG.trueRightHandModelPearl2[1]:
                    #-#print("1RightModel Closer than True one!!:\nrightHandModel =", rightHandModel, "\nVG.trueRightHandModelPearl2 =", VG.trueRightHandModelPearl2)
                    if VG.flagWarning:
                        VG.masterACoefficient = float('inf')
                    else:
                        VG.masterACoefficient = Droite_dInteret[0][0]
                    VG.counterPermanence2 = 0
                    VG.flagInitModels2 = False

            elif flagFoundModelsRight and VG.trueRightHandModelPearl2: # and VG.trueSecondRightModel:
                #&#print("I've entered: flagFoundModelsRight and VG.trueRightHandModelPearl2 and VG.trueSecondRightModel")
                if (abs(VG.trueRightHandModelPearl2[0] - rightHandModel[0]) <= 0.5): # (abs(abs(rightSecondModel[1] - VG.trueSecondRightModel[1])/VG.trueSecondRightModel[1]) <= 0.6) and
                    #&#print("I've entered VG.trueRightHandModelPearl2 and (abs(VG.trueRightHandModelPearl2[0] - rightHandModel[0]) <= 0.3)")
                    #-#if rightHandModel[1] < VG.trueRightHandModelPearl2[1]:
                    #-#print("2RightModel Closer than True one!!:\nrightHandModel =", rightHandModel,"\nVG.trueRightHandModelPearl2 =", VG.trueRightHandModelPearl2)
                    VG.trueRightHandModelPearl2 = rightHandModel  # Added
                    VG.trueRightModelDataPearl2 = Clustered_data[index]  # Added
                    VG.trueSecondRightModel = rightSecondModel
                    VG.trueSecondRightData = rightSecondData
                    Data_dInteret = [VG.trueRightModelDataPearl2, VG.trueSecondRightData]
                    Droite_dInteret = [VG.trueRightHandModelPearl2, VG.trueSecondRightModel]
                    VG.minimalBCoefficient = Droite_dInteret[0][1]

                    if VG.flagWarning:
                        VG.masterACoefficient = float('inf')
                    else:
                        VG.masterACoefficient = Droite_dInteret[0][0]
                    VG.counterPermanence2 = 0
                else:
                    #&#print("keeping the previous models")
                    Data_dInteret = [VG.trueRightModelDataPearl2, VG.trueSecondRightData]
                    Droite_dInteret = [VG.trueRightHandModelPearl2, VG.trueSecondRightModel]
                    VG.minimalBCoefficient = Droite_dInteret[0][1]
                    if VG.flagWarning:
                        VG.masterACoefficient = float('inf')
                    else:
                        VG.masterACoefficient = Droite_dInteret[0][0]
                    VG.counterPermanence2 = VG.counterPermanence2 + 1

            elif not flagFoundModelsRight and VG.trueRightHandModelPearl2 and not VG.trueSecondRightModel:
                #&#print("## I have a true model, but not two parallel ones on the new reading testing if the new model is parallel to the old one...  ##")
                if (abs(VG.trueRightHandModelPearl2[0] - rightHandModel[0]) <= 0.3 ):# and (abs(abs(rightHandModel[1] - VG.trueRightHandModelPearl2[1]) / VG.trueRightHandModelPearl2[1]) <= 0.5):
                    #&#print("Yes, they are parallel and not so far from each other!")
                    #-#if rightHandModel[1] < VG.trueRightHandModelPearl2[1]:
                    #-#print("3RightModel Closer than True one!!:\nrightHandModel =", rightHandModel,"\nVG.trueRightHandModelPearl2 =", VG.trueRightHandModelPearl2)
                    VG.trueRightHandModelPearl2 = rightHandModel  # Added
                    VG.trueRightModelDataPearl2 = Clustered_data[index]  # Added
                    Data_dInteret = [VG.trueRightModelDataPearl2]
                    Droite_dInteret = [VG.trueRightHandModelPearl2]
                    VG.minimalBCoefficient = Droite_dInteret[0][1]

                    if VG.flagWarning:
                        VG.masterACoefficient = float('inf')
                    else:
                        VG.masterACoefficient = Droite_dInteret[0][0]
                    VG.counterPermanence2 = 0
                else:
                    #&#print("NewModel is not parallel enough")
                    Data_dInteret = [VG.trueRightModelDataPearl2]
                    Droite_dInteret = [VG.trueRightHandModelPearl2]
                    VG.minimalBCoefficient = Droite_dInteret[0][1]
                    if VG.flagWarning:
                        VG.masterACoefficient = float('inf')
                    else:
                        VG.masterACoefficient = Droite_dInteret[0][0]
                    VG.counterPermanence2 = VG.counterPermanence2 + 1

            elif not flagFoundModelsRight and VG.trueRightHandModelPearl2 and VG.trueSecondRightModel:
                #&#print("I've entered: not flagFoundModelsRight and VG.trueRightHandModelPearl2 and VG.trueSecondRightModel")
                if rightSecondModel and (abs(VG.trueRightHandModelPearl2[0] - rightHandModel[0]) <= 0.3 ):
                    #&#print("rightSecondModel and (abs(VG.trueRightHandModelPearl2[0] - rightHandModel[0]) <= 0.3 )")
                    VG.trueRightHandModelPearl2 = rightHandModel  # Added
                    VG.trueRightModelDataPearl2 = Clustered_data[index]  # Added
                    VG.trueSecondRightModel = rightSecondModel
                    VG.trueSecondRightData = rightSecondData
                    Data_dInteret = [VG.trueRightModelDataPearl2, VG.trueSecondRightData]
                    Droite_dInteret = [VG.trueRightHandModelPearl2, VG.trueSecondRightModel]
                    #-#if rightHandModel[1] < VG.trueRightHandModelPearl2[1]:
                    #-#print("4RightModel Closer than True one!!:\nrightHandModel =", rightHandModel,"\nVG.trueRightHandModelPearl2 =", VG.trueRightHandModelPearl2)

                    VG.minimalBCoefficient = Droite_dInteret[0][1]
                    VG.counterPermanence2 = 0

                elif not rightSecondModel and (abs(VG.trueRightHandModelPearl2[0] - rightHandModel[0]) <= 0.3 ):
                    #&#print("not rightSecondModel and (abs(VG.trueRightHandModelPearl2[0] - rightHandModel[0]) <= 0.3 )")
                    #-#if rightHandModel[1] < VG.trueRightHandModelPearl2[1]:
                    #-#print("5RightModel Closer than True one!!:\nrightHandModel =", rightHandModel,"\nVG.trueRightHandModelPearl2 =", VG.trueRightHandModelPearl2)
                    VG.trueRightHandModelPearl2 = rightHandModel  # Added
                    VG.trueRightModelDataPearl2 = Clustered_data[index]  # Added
                    VG.trueSecondRightModel = []
                    VG.trueSecondRightData = []
                    Data_dInteret = [VG.trueRightModelDataPearl2]
                    Droite_dInteret = [VG.trueRightHandModelPearl2]

                    VG.minimalBCoefficient = Droite_dInteret[0][1]
                    VG.counterPermanence2 = 0
                else:
                    #&#print("Went through all conditions, still exception, keeping the old models")
                    Data_dInteret = [VG.trueRightModelDataPearl2, VG.trueSecondRightData]
                    Droite_dInteret = [VG.trueRightHandModelPearl2, VG.trueSecondRightModel]
                    VG.minimalBCoefficient = Droite_dInteret[0][1]
                    if VG.flagWarning:
                        VG.masterACoefficient = float('inf')
                    else:
                        VG.masterACoefficient = Droite_dInteret[0][0]
                    VG.counterPermanence2 = VG.counterPermanence2 + 1

            elif VG.trueSecondRightModel and not flagFoundModelsRight and not VG.trueRightHandModelPearl2:
                print("I'm at VG.trueSecondRightModel and not flagFoundModelsRight and not VG.trueRightHandModelPearl2")

            #else:
                #print("I've fallen into the general else, current situation is:")
                #&#print("VG.trueSecondRightModel = ",VG.trueSecondRightModel)
                #&#print("trueRightHandModel = ",VG.trueRightHandModelPearl2)
                #&#print("flagFoundModelsRight = ",flagFoundModelsRight)
                #&#print("rightHandModel = ",rightHandModel)
                #&#print("rightSecondModel = ",rightSecondModel)

                #-#if rightHandModel and VG.trueRightHandModelPearl2:
            #-#print("rightHandModel= ",rightHandModel,"\nVG.trueRightHandModelPearl2[1]= ",VG.trueRightHandModelPearl2)
            #-#if rightHandModel[1] < VG.trueRightHandModelPearl2[1]:
        #-#print("6RightModel Closer than True one!!:\nrightHandModel =",rightHandModel,"\nVG.trueRightHandModelPearl2 =",VG.trueRightHandModelPearl2)

        elif leLeftModels and not leRightModels:# and (VG.directionTurn == "Right"):
            #&#print("Entered: LeftHandModel and (not leRightModels)")
            # print("The true models are:\nright Hand = ", VG.trueRightHandModelPearl2, "\nleft Hand = ", VG.trueLeftHandModelPearl2)
            flagFoundModelsLeft = False
            #VG.trueRightModelDataPearl2 = []
            #VG.trueRightHandModelPearl2 = []

            leftSecondModel = []
            leftSecondData = []

            #-#print("leLeft ModelsBefore sort method", leLeftModels)
            leLeftModels.sort(key=lambda x: x[1])
            #-#print("leLeft Models After sort method", leLeftModels)

            leftHandModel = leLeftModels[0]
            # leftHandData

            for k in range(len(leLeftModels)):
                # print("Executing the for...", k)
                if (abs(leftHandModel[0] - leLeftModels[k][0]) <= 0.1) and (leftHandModel != leLeftModels[k]):
                    # print("I have two parallel Models in leLeftModels, they are:",leftHandModel,"and\n",leLeftModels[k])
                    leftSecondModel = leLeftModels[k]
                    leftSecondData = leLeftData[k]
                    flagFoundModelsLeft = True

            if VG.flagInitModels3 and flagFoundModelsLeft:
                #&#print("On Initialization")
                VG.trueLeftHandModelPearl2 = leftHandModel
                VG.trueLeftModelDataPearl2 = Clustered_data[index2]
                VG.trueSecondLeftModel = leftSecondModel
                VG.trueSecondLeftData = leftSecondData
                """Data_dInteret = [VG.trueLeftModelDataPearl2, VG.trueSecondLeftData]
                Droite_dInteret = [VG.trueLeftHandModelPearl2, VG.trueSecondLeftModel]
                VG.minimalBCoefficient = Droite_dInteret[0][1]
                if VG.flagWarning:
                    VG.masterACoefficient = float('inf')
                else:
                    VG.masterACoefficient = Droite_dInteret[0][0]"""
                VG.flagInitModels3 = False

            elif VG.flagInitModels3 and not flagFoundModelsLeft:
                VG.trueLeftHandModelPearl2 = leftHandModel
                VG.trueLeftModelDataPearl2 = Clustered_data[index2]
                """Data_dInteret = [VG.trueLeftModelDataPearl2]
                Droite_dInteret = [VG.trueLeftHandModelPearl2]
                VG.minimalBCoefficient = Droite_dInteret[0][1]
                if VG.flagWarning:
                    VG.masterACoefficient = float('inf')
                else:
                    VG.masterACoefficient = Droite_dInteret[0][0]"""
                VG.flagInitModels3 = False

            elif flagFoundModelsLeft and VG.trueLeftHandModelPearl2 and VG.trueSecondLeftModel:
                #&#print("I've entered: flagFoundModelsLeft and VG.trueLeftHandModelPearl2 and VG.trueSecondLeftModel")
                if (abs(VG.trueLeftHandModelPearl2[0] - leftHandModel[0]) <= 0.5):  # (abs(abs(leftSecondModel[1] - VG.trueSecondLeftModel[1])/VG.trueSecondLeftModel[1]) <= 0.6) and
                    #&#print("I've entered VG.trueLeftHandModelPearl2 and (abs(VG.trueLeftHandModelPearl2[0] - leftHandModel[0]) <= 0.3)")
                    VG.trueLeftHandModelPearl2 = leftHandModel  # Added
                    VG.trueLeftModelDataPearl2 = Clustered_data[index2]  # Added
                    VG.trueSecondLeftModel = leftSecondModel
                    VG.trueSecondLeftData = leftSecondData
                    """Data_dInteret = [VG.trueLeftModelDataPearl2, VG.trueSecondLeftData]
                    Droite_dInteret = [VG.trueLeftHandModelPearl2, VG.trueSecondLeftModel]
                    VG.minimalBCoefficient = Droite_dInteret[0][1]
                    if VG.flagWarning:
                        VG.masterACoefficient = float('inf')
                    else:
                        VG.masterACoefficient = Droite_dInteret[0][0]"""
                else:
                    #&#print("keeping the previous models")
                    """Data_dInteret = [VG.trueLeftModelDataPearl2, VG.trueSecondLeftData]
                    Droite_dInteret = [VG.trueLeftHandModelPearl2, VG.trueSecondLeftModel]
                    VG.minimalBCoefficient = Droite_dInteret[0][1]
                    if VG.flagWarning:
                        VG.masterACoefficient = float('inf')
                    else:
                        VG.masterACoefficient = Droite_dInteret[0][0]"""

            elif not flagFoundModelsLeft and VG.trueLeftHandModelPearl2 and not VG.trueSecondLeftModel:
                #&#print("## I have a true model, but not two parallel ones on the new reading testing if the new model is parallel to the old one...  ##")
                if (abs(VG.trueLeftHandModelPearl2[0] - leftHandModel[
                    0]) <= 0.3):  # and (abs(abs(leftHandModel[1] - VG.trueLeftHandModelPearl2[1]) / VG.trueLeftHandModelPearl2[1]) <= 0.5):
                    # print("Yes, they are parallel and not so far from each other!")
                    VG.trueLeftHandModelPearl2 = leftHandModel  # Added
                    VG.trueLeftModelDataPearl2 = Clustered_data[index2]  # Added
                    """Data_dInteret = [VG.trueLeftModelDataPearl2]
                    Droite_dInteret = [VG.trueLeftHandModelPearl2]
                    VG.minimalBCoefficient = Droite_dInteret[0][1]
                    if VG.flagWarning:
                        VG.masterACoefficient = float('inf')
                    else:
                        VG.masterACoefficient = Droite_dInteret[0][0]"""
                else:
                    #&#print("NewModel is not parallel enough")
                    """Data_dInteret = [VG.trueLeftModelDataPearl2]
                    Droite_dInteret = [VG.trueLeftHandModelPearl2]
                    VG.minimalBCoefficient = Droite_dInteret[0][1]
                    if VG.flagWarning:
                        VG.masterACoefficient = float('inf')
                    else:
                        VG.masterACoefficient = Droite_dInteret[0][0]"""

            elif not flagFoundModelsLeft and VG.trueLeftHandModelPearl2 and VG.trueSecondLeftModel:
                #&#print("I've entered: not flagFoundModelsLeft and VG.trueLeftHandModelPearl2 and VG.trueSecondLeftModel")
                if leftSecondModel and (abs(VG.trueLeftHandModelPearl2[0] - leftHandModel[0]) <= 0.3):
                    #&#print("leftSecondModel and (abs(VG.trueLeftHandModelPearl2[0] - leftHandModel[0]) <= 0.3 )")
                    VG.trueLeftHandModelPearl2 = leftHandModel  # Added
                    VG.trueLeftModelDataPearl2 = Clustered_data[index2]  # Added
                    VG.trueSecondLeftModel = leftSecondModel
                    VG.trueSecondLeftData = leftSecondData
                    """Data_dInteret = [VG.trueLeftModelDataPearl2, VG.trueSecondLeftData]
                    Droite_dInteret = [VG.trueLeftHandModelPearl2, VG.trueSecondLeftModel]
                    VG.minimalBCoefficient = Droite_dInteret[0][1]"""
                elif not leftSecondModel and (abs(VG.trueLeftHandModelPearl2[0] - leftHandModel[0]) <= 0.3):
                    #&#print("not leftSecondModel and (abs(VG.trueLeftHandModelPearl2[0] - leftHandModel[0]) <= 0.3 )")
                    VG.trueLeftHandModelPearl2 = leftHandModel  # Added
                    VG.trueLeftModelDataPearl2 = Clustered_data[index2]  # Added
                    VG.trueSecondLeftModel = []
                    VG.trueSecondLeftData = []
                    """Data_dInteret = [VG.trueLeftModelDataPearl2]
                    Droite_dInteret = [VG.trueLeftHandModelPearl2]
                    VG.minimalBCoefficient = Droite_dInteret[0][1]"""
                else:
                    #&#print("Went through all conditions, still exception, keeping the old models")
                    """Data_dInteret = [VG.trueLeftModelDataPearl2, VG.trueSecondLeftData]
                    Droite_dInteret = [VG.trueLeftHandModelPearl2, VG.trueSecondLeftModel]
                    VG.minimalBCoefficient = Droite_dInteret[0][1]
                if VG.flagWarning:
                    VG.masterACoefficient = float('inf')
                else:
                    VG.masterACoefficient = Droite_dInteret[0][0]"""

            elif VG.trueSecondLeftModel and not flagFoundModelsLeft and not VG.trueLeftHandModelPearl2:
                print("I'm at VG.trueSecondLeftModel and not flagFoundModelsLeft and not VG.trueLeftHandModelPearl2")

        # print("The true models are:\nleft Hand = ",VG.trueLeftHandModelPearl2,"\nright Hand = ",VG.trueRightHandModelPearl2)
        # print("The Models that arrived for left and right hand were:\nleft Hand = ",leftHandModel,"\nright Hand = ",rightHandModel)
        # print("Data_dInteret=", Data_dInteret, "\nDroite_dInteret =", Droite_dInteret)
        if not (Droite_dInteret and Data_dInteret):
        #-#print("Jumping the writing...")
        #-#else:
            if VG.trueLeftModelDataPearl2 and VG.trueRightModelDataPearl2:
                Data_dInteret = [VG.trueLeftModelDataPearl2, VG.trueRightModelDataPearl2]
                Droite_dInteret = [VG.trueLeftHandModelPearl2, VG.trueRightHandModelPearl2]
                TypeRang = "Null"
            elif VG.trueLeftModelDataPearl2 and not VG.trueRightModelDataPearl2:
                Data_dInteret = [VG.trueLeftModelDataPearl2]
                Droite_dInteret = [VG.trueLeftHandModelPearl2]
                TypeRang = "Null"
            elif VG.trueRightModelDataPearl2 and not VG.trueLeftModelDataPearl2:
                Data_dInteret = [VG.trueRightModelDataPearl2]
                Droite_dInteret = [VG.trueRightHandModelPearl2]
                VG.minimalBCoefficient = Droite_dInteret[0][1]
                TypeRang = "Null"

        if VG.flagWarning:
            VG.masterACoefficient = float('inf')

        if len(Data_dInteret) == 2:
            TypeRang = "Double"
            # print("Coef b doite : " + str(Droite_dInteret[0][1]) + "\tCoef b gauche : " + str(Droite_dInteret[1][1]) + "\tDistance inter-rang : " + str(abs(Droite_dInteret[0][1]) + abs(Droite_dInteret[1][1])))
        elif len(Data_dInteret) == 1:
            if Droite_dInteret[0][1] > 0:
                TypeRang = "Droite"
                # print("Ecartement instantané : " + str(Droite_dInteret[0][1]))
            else:
                TypeRang = "Gauche"
                # print("Ecartement instantané : " + str(Droite_dInteret[0][1]))
        else:
            TypeRang = "Null"

            #-#print("Data_dInteret = ", Data_dInteret, "\nDroite_dInteret = ", Droite_dInteret)

        return Data_dInteret, Droite_dInteret, TypeRang
    # -----------------------------------------------------------------------------------------------
    # -----------------------------------------------------------------------------------------------
    # --################################  END OF USED FUNCTIONS. ##################################--
    # -----------------------------------------------------------------------------------------------
    # -----------------------------------------------------------------------------------------------
    def modelsKillInit(self, labels, models, threshParallel, threshRatio):
        print("\n--> Entering modelsKillInit...")
        modelsLabels = []
        modelsToBeDeleted = []
        bestRatio = float('inf')
        nbAntModels = len(models)
        # print("\nmodels before energy division:",models)
        # print("safeModels = ",safeModels)

        safeModels = self.findParallels2(models, threshParallel)

        # counterModels = np.zeros(len(models))

        for model in range(len(models)):
            modelsLabels.append(models[model][2])

        for k in range(len(safeModels)):

            try:
                index = modelsLabels.index(safeModels[k])
            except ValueError:
                index = -2

            if (index != -2):
                models[index][4] = models[index][4] / 10
                # counterModels[index] = counterModels[index] + 1
        # print("\nmodels after energy division:",models)
        # print(counterModels)

        for model in range(len(models)):
            if (models[model][4] / models[model][3] < bestRatio):
                bestModel = models[model]
                bestRatio = models[model][4] / models[model][3]
            if not (models[model][4] / models[model][3] <= threshRatio):
                modelsToBeDeleted.append(int(models[model][2]))

        # print("The list of models to be deleted is:",modelsToBeDeleted)

        for i in range(len(modelsToBeDeleted)):

            for j in range(len(models)):
                if (models[j][2] == (modelsToBeDeleted[i])):
                    del models[j]
                    for k in range(len(labels)):
                        if (labels[k] == modelsToBeDeleted[i]) and (labels[k] != bestModel[2]):
                            labels[k] = 0
                    break
        if (len(models) == 0):
            # print("######## THERE WAS AN ERROR IN modelsKillInit ########")
            # print("The best model we had was:",bestModel,"and it has been reappended to models")
            models.append(bestModel)
        # print((nbAntModels - len(models)),"model(s) was(were) destroyed here")
        # print("The new list of Models is:",models)
        #print("<-- Leaving modelsKillInit...")
        return models, labels
    # -----------------------------------------------------------------------------------------------
    # Plotting the results of PEARL (Unused for the robot)
    def plotAll(self, models, labels, x_data, y_data, initModels, realNbInit, randomlyChosenX, randomlyChosenY,nbOfInitModels, nbMinPoints, nbOfIteractions, savedModels, savedLabels):
        fig, (ax1, ax2, ax3, ax4) = plt.subplots(nrows=4, sharex=True)

        ax1.scatter(np.array(x_data), np.array(y_data))
        # ax1.set_title("Samples")

        survivingModels = []
        modelsLabels = []
        numbOfIn = 0
        numbOfInSaved = 0
        for k in range(len(models)):
            survivingModels.append(int(models[k][2]))
            numbOfIn = numbOfIn + models[k][3]

        for k in range(len(savedModels)):
            numbOfInSaved = numbOfInSaved + savedModels[k][3]

        inOutRatio = numbOfIn / len(x_data)
        inOutRatioSaved = numbOfInSaved / len(x_data)

        fig.suptitle('Number of initial samples = ' + str(len(x_data)) + '\nNumber of initial models ' + str(
            nbOfInitModels) + '\nNumber of final models = ' + str(len(models)) + '\nNumber of iteractions = ' + str(
            nbOfIteractions), fontsize=10, fontweight='bold')

        # models,labels = resetModelsLabels(models,labels)

        number = len(models)
        cmap = plt.get_cmap('Vega20b')
        colors = [cmap(i) for i in np.linspace(0, 1, number)]

        t = self.findt(models, labels, x_data, y_data)
        # Printing Final Models
        for i, color in enumerate(colors, start=1):
            currentModelX = []
            currentModelY = []
            for k in range(len(labels)):
                if (labels[k] == models[i - 1][2]):
                    currentModelX.append(float(x_data[k]))
                    currentModelY.append(float(y_data[k]))

            ax3.scatter(currentModelX, currentModelY, color=color)
            ax3.plot(t, float(models[i - 1][0]) * t + float(models[i - 1][1]), color=color,
                     label='Model ' + str(int(models[i - 1][2])))
        ax3.legend(loc='best')
        inOutRatio = round(inOutRatio, 3)
        ax3.set_title("Last calculated models, inlier/outlier =" + str(inOutRatio))

        # Printing Initial Models
        ax2.scatter(randomlyChosenX, randomlyChosenY, color='black')
        number2 = len(initModels)
        cmap = plt.get_cmap('Vega20b')
        colors = [cmap(i) for i in np.linspace(0, 1, number2)]

        for i, color in enumerate(colors, start=1):
            ax2.plot(t, float(initModels[i - 1][0]) * t + float(initModels[i - 1][1]), color=color,
                     label='initModel' + str(initModels[i - 1][2]).format(i=i))
        ax2.set_title("Initial Points and Models")

        # Printing Saved Models
        number = len(savedModels)
        cmap = plt.get_cmap('Vega20b')
        colors = [cmap(i) for i in np.linspace(0, 1, number)]

        t2 = self.findt(savedModels, savedLabels, x_data, y_data)
        # Printing Final savedModels
        for i, color in enumerate(colors, start=1):
            currentModelX = []
            currentModelY = []
            for k in range(len(savedLabels)):
                if (savedLabels[k] == savedModels[i - 1][2]):
                    currentModelX.append(float(x_data[k]))
                    currentModelY.append(float(y_data[k]))

            ax4.scatter(currentModelX, currentModelY, color=color)
            ax4.plot(t2, float(savedModels[i - 1][0]) * t2 + float(savedModels[i - 1][1]), color=color,
                     label='Model ' + str(int(savedModels[i - 1][2])))
            ax1.plot(t2, float(savedModels[i - 1][0]) * t2 + float(savedModels[i - 1][1]), color=color,
                     label='Model ' + str(int(savedModels[i - 1][2])))
        ax4.legend(loc='best')
        inOutRatioSaved = round(inOutRatioSaved, 3)
        ax4.set_title("Saved models, inliers/outliers=" + str(inOutRatioSaved))

        plt.show()
        # -----------------------------------------------------------------------------------------------
    # -----------------------------------------------------------------------------------------------
    # Unused, this is meant to be used for ploting the results of PEARL as graphs.
    def findt(self, models, labels, x_data, y_data):
        maxTemp = float('-inf')
        minTemp = float('inf')

        for model in range(len(models)):
            currentModelX = []
            currentModelY = []
            for k in range(len(labels)):
                if (labels[k] == models[model][2]):
                    currentModelX.append(float(x_data[k]))
                    currentModelY.append(float(y_data[k]))
            if (max(currentModelX) > maxTemp):
                maxTemp = float(max(currentModelX))
            if (min(currentModelX) < minTemp):
                minTemp = float(min(currentModelX))

        t = np.arange(minTemp, maxTemp, 20)
        return t
    # -----------------------------------------------------------------------------------------------
    # Unused, auxiliar function for ploting the results of PEARL
    def resetModelsLabels(self, models, labels):
        labelsModels = []
        newModels = []
        for k in range(len(models)):
            labelsModels.append(models[k][2])

        for element in range(len(labelsModels)):
            for k in range(len(models)):
                if (labelsModels[element] == models[k][2]):
                    a = models[k][0]
                    b = models[k][1]
                    c = element + 1
                    d = models[k][3]
                    e = models[k][4]
                    for label in range(len(labels)):
                        if (labels[label] == models[k][2]):
                            labels[label] = 100 + element + 1
                    newModels.append([a, b, c, d, e])

        for label in range(len(labels)):
            if (labels[label] > 100):
                labels[label] = labels[label] - 100

        return newModels, labels
