from threading import Lock

LOCK = Lock()  # for the thread shared variables
LiDAR = [[0]*2 for i in range(0, 271)]  # 271 measurements, LiDAR[i][0]=distance, LiDAR[i][1]=angle

Mode_Automatique = False
ParcoursRangee = False
DemiTours = [False]
ChercheRang = [False]

speedRightMotor = 0
speedLeftMotor = 0

speedTerminal = 127
speedMax = 60

# PEARL1 VARIABLES
flagFirst = True
flagfff = False
kCoefThresh = 0.5
bCoefThresh = 350
threshParallel = 0.2
threshRatio = 2.5
distOutlier = 150
delta = 15
nbMaxIteractions = 6
alpha = 1000
trueRightHandModel = []
trueLeftHandModel = []
trueLeftModelData = []
trueRightModelData = []
flagInitModels = True
counterPermanence = 0
lineHistory = []
rightHandAverage = 0
rightHandSum = 0
leftHandAverage = 0
leftHandSum = 0
rightHandDist = 0
leftHandDist = 0
rightHandDistSum = 0
leftHandDistSum = 0
distSum = []
averageDist = 0
targetDistAvg = 0
targetDistSum = []
distToTarget = 0
distToLeft = []
distToRight = []
averageDistToRight = 0
averageDistToLeft = 0
defaultDistance = 400
HistoriqueFiltre2 = []

# Changing rows variables:
repositionOnRow = False
turningRow = False
flagUsePearl2 = False
finishedTurn = False
searchingRow = False
flagWarning = False
reachedMiddleRow = False
enteringNewRow = False
fallBack = False
repositionComplete = False
flagLastRow = False
stablePosition = False
directionTurn = "Left" #directionTurn = "Right" #

# PEARL 2 Variables:
counterPermanence2= 0
trueRightHandModelPearl2 = []
trueLeftHandModelPearl2 = []
trueLeftModelDataPearl2 = []
trueRightModelDataPearl2 = []

flagInitModels2 = True
flagInitModels3 = True
trueSecondRightData = []
trueSecondRightModel = []
trueSecondLeftData = []
trueSecondLeftModel = []
minimalBCoefficient = float('inf')
masterACoefficient = float('inf')


HistoriqueRang = [0, 0, 0]
Analyse_count = []
HistoriqueErreur = []
HistoriqueFiltre = []
HistoriqueMagneto = []
HistoriqueGPS = []

#timeGPS = 0
distMarcheX = 0
distMarcheY = 0

# Tracking variables:
# Accelerometer variables:
AccX = []
AccY = []
AccZ = []
HistoriqueAccel = []
timeAccel = []
currentXSpeed = 0
currentYSpeed = 0
spaceX = 0
spaceY = 0
numOfReads = 0
lenRows = []
yRows = []

# Gyro Variables:
numOfReadsGyro = 0
timeGyro = []
HistoriqueGyro = []
angleX = 0
angleY = 0
angleZ = 0

# Accurate tracking variables:
Rest = []
flagReadAccel = False
flagReadGyro = False
angleX = 0
historyAngleX = []

# Testing box:
data_x_tmp = []
data_y_tmp = []
currentModels = []
currentData = []
currentCRModel = []
currentCRModelData = []
boxRight = []
boxLeft = []

# Inertial Central:
IMUStatus = 0
theta = 0
posx = 0
posy = 0
resetIMU = False
x_endRow = float('inf')
y_endRow = float('inf')
x_nextRow = float('inf')
y_nextRow = float('inf')

Row_param =[]

HistoriquePtsTransverses = []