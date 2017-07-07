from threading import Thread
from threading import Event as thEvent
import socket as socket
from global_variables import LiDAR
from global_variables import LOCK
import global_variables as VG
import struct
import time


class LiDARThread(Thread):
    # To update the LiDAR measurements as fast as possible and send data to motors.
    def __init__(self, ip):
        Thread.__init__(self)
        self._stop = thEvent()
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # the socket to get the data from the robot
        self._ip = ip  # IP Address of the robot
        self._port = 5555       # the corresponding API port
        self._lidarID = 0xAA    # the id of the lidar data
        self._motorsID = 0xA8   # The ID of the Motor commands

        self._socket2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._statusID = 0xA6
        self._commandID = 0xA7
        self._sizeIMUAcBat = 1
        self._sizeThePosDist = 8
        self._sizeOdom = 4
        self._sizeMag = 2
    # ------------------------------------------------------------------------------------------------------------------
    def run(self):
        try:
            self._socket.connect((self._ip, self._port))
            #self._socket2.connect((self._ip, self._port))
            print("Connected to LiDAR and Motor")
            while not self._stop.isSet():
                self.sendData2Mot()
                self.getsensorvalues()
                if VG.resetIMU:
                    self.resetIMU()
                    VG.resetIMU = False
        except socket.error as msg:
            # if an error occured, thoses messages will be displayed
            print("Error - connect_socket - DisplayRobotSensors")
            print("Error Code: " + str(msg[0]) + " - Message: " + msg[1])
    # ------------------------------------------------------------------------------------------------------------------
    # To update the LiDAR measurements as fast as possible
    def getsensorvalues(self): # Rèmy's Function.
        # this function gets the LiDAR data from the robot and computes the corresponding distances

        buf = 1024  # maximal size of the recovered data from the socket
        try:
            # get data from the robot
            response = self._socket.recv(buf)
            size = len(response)

            # loop over the data recovered
            for i in range(0, size - 11):  # size-11 because we want a NAIO01 frame (header+id+payload)
                if response[i:i + 6] == b"NAIO01" and response[i + 6] == self._lidarID:  # if we found the header and the Id corresponds to the LiDAR data

                    # we compute the corresponding payload (must be 542)
                    payload = response[i + 7] * 2048 + response[i + 8] * 1024 + response[i + 9] * 256 + response[i + 10]

                    if size - i - 11 > payload:  # check if all the Lidar data are in the received data
                        #del self._distances[:]  # we erase the previously revorded data set

                        LOCK.acquire()
                        for j in range(0, payload, 2):
                            # convert the received data to distances
                            # the loop is done 2x2 steps, because we need the MSB and the LSB to compute the distance
                            #   -> note that distances are given over 16bits while a data correspond to 8bits (1 byte)
                            #$$# self._distances.append([response[j + i + 11] * 0xFF + response[i + j + 12], j / 2]) # Remy
                            LiDAR[int(j / 2)][0] = response[j + i + 11] * 0xFF + response[i + j + 12]
                            LiDAR[int(j / 2)][1] = int(j / 2)

                        LOCK.release()

                    # self.draw()  # ask to redraw the window to update the lidar measurements display (Remy)
                    break  # if a LiDAR data has been found, we stop the loop
                    # NOTE: this is not sexy at all, must be improved! do not put a break into a for loop...
            # this function is called again after a 50ms delay
            #self.getsensorvalues()
            for i in range(0, size - 11):  # size-11 because we want a NAIO01 frame (header+id+payload)

                if response[i:i + 6] == b"NAIO01" and response[i + 6] == self._statusID:  # if we found the header and the Id corresponds to the LiDAR data

                    payload = response[i + 7] * 2048 + response[i + 8] * 1024 + response[i + 9] * 256 + response[i + 10]

                    #print("size of payload = ",payload)

                    if size - i - 11 > payload:  # check if all the Status data are in the received data
                        #print("########## IMU ##########")#\nsize =",size," | Payload = ",payload )
                        #print(response)
                        #for b in response:
                            #print(hex(b))

                        posinic = i+11
                        posfim = posinic + self._sizeIMUAcBat

                        #print(posinic)

                        IMU = int(response[posinic])

                        posinic = posfim
                        posfim = posinic + self._sizeThePosDist

                        theta2, = struct.unpack_from('d', response, offset = posinic)
                        theta = float(theta2)
                        posx, posy = struct.unpack_from('dd', response,offset=(posfim+16))  # [0]#[36]self.convertBytes2(response[i+36:i+52])

                        #print("IMU = ", IMU, "Theta2 = ", float(theta2), "(posx, posy) = (", posx, ",", posy, ")")

                        LOCK.acquire()
                        VG.IMUStatus = IMU
                        VG.theta = float(theta2)
                        VG.posx = posx
                        VG.posy = posy
                        LOCK.release()
                    break

        except socket.error as msg:
            # if the socket is no longer connected
            print("Error - getsensorvalues - DisplayRobotSensors")
            print("Connect failed. Error Code: " + str(msg[0]) + " - Message: " + msg[1])
    # ------------------------------------------------------------------------------------------------------------------
    # To send data to motors as fast as possible
    def sendData2Mot(self): # Sending data to the motors as fast as possible in order to avoid timeouts

        LOCK.acquire()
        rightWheelSpeed = VG.speedRightMotor
        leftWheelSpeed = VG.speedLeftMotor
        LOCK.release()

        if leftWheelSpeed < 0:
            leftWheelSpeed = int(hex((leftWheelSpeed + (1 << 8)) % (1 << 8)), 16)
        if rightWheelSpeed < 0:
            rightWheelSpeed = int(hex((rightWheelSpeed + (1 << 8)) % (1 << 8)), 16)

        request = bytearray([ord('N'), ord('A'), ord('I'), ord('O'), ord('0'), ord('1'), self._motorsID, 0x00, 0x00, 0x00, 0x02, int(leftWheelSpeed), int(rightWheelSpeed), 0x00, 0x00, 0x00, 0x00])

        try:
            self._socket.send(request)
        except socket.error as msg:
            print("Error - run MoveRobotWithKeyBoard")
            print('Error Code : ' + str(msg[0]) + ' - Message ' + msg[1])
            LOCK.acquire()
            VG.speedRightMotor = 0
            VG.speedLeftMotor = 0
            LOCK.release()
    # ------------------------------------------------------------------------------------------------------------------
    def getStats(self):

        # to avoid to be disconnected it is needed to send a watchod data
        # LOCK.acquire()
        # request = bytearray([ord('N'), ord('A'), ord('I'), ord('O'), ord('0'), ord('1'), 0xB4, 0x00])
        # self._socket.send(request)  # sending the watchdog data
        # LOCK.release()
        buf = 256 # maximal size of the recovered data from the socket
        try:

            #self._socket.connect((self._ip, self._port))
            #response2 = []
            #response3 = []
            response = 0
            response = self._socket2.recv(buf)
            size = len(response)
            #print(response)

            for i in range(0, size - 11):  # size-11 because we want a NAIO01 frame (header+id+payload)

                if response[i:i + 6] == b"NAIO01" and response[i + 6] == self._statusID:  # if we found the header and the Id corresponds to the LiDAR data

                    payload = response[i + 7] * 2048 + response[i + 8] * 1024 + response[i + 9] * 256 + response[i + 10]

                    #print("size of payload = ",payload)

                    if size - i - 11 > payload:  # check if all the Status data are in the received data
                        print("\n##########Start##########\nsize =",size," | Payload = ",payload )

                        #print(response)
                        #for b in response:
                            #print(hex(b))

                        posinic = i+11
                        posfim = posinic + self._sizeIMUAcBat

                        #print(posinic)

                        IMU = int(response[posinic])
                        print("IMU = ", IMU)
                        posinic = posfim
                        posfim = posinic + self._sizeThePosDist

                        theta2, = struct.unpack_from('d', response, offset = posinic)
                        theta = float(theta2)
                        print("Theta2 = ", float(theta2))
                        posx, posy = struct.unpack_from('dd', response,offset=(posfim+16))  # [0]#[36]self.convertBytes2(response[i+36:i+52])

                        print("(posx, posy) = (",posx,"," ,posy,")")

                        LOCK.acquire()
                        VG.IMUStatus = IMU
                        VG.theta = float(theta2)
                        VG.posx = posx
                        VG.posy = posy
                        LOCK.release()

                    break

                    #posinic = posfim

                        #[MagX, MagY, MagY] = struct.unpack_from('hhh', response,offset = posinic)

                        #print(response[i+36:i+52])
                        #posx, posy = struct.unpack_from('dd',response, offset = 35+i)#[0]#[36]self.convertBytes2(response[i+36:i+52])
                        #dist = self.convertBytes(response[i+53:i+60])
                        #actuPos = int(response[i+61])

                        #[MagX, MagY, MagY] = self.convertBytes3(response[i+63:i+68])
                        #batteryStat = int(response[62])
                        #print("IMU = ", IMU)
                        #print("Theta = ", theta)
                        #print("Theta2 = ", int(theta2))
                        #print("Theta3 = ", theta3)
                        #print("posi = [",posx,",",posy,"]")
                        #print("odoFR, odoRR, odoRL, odoFL = ",odoFR, odoRR, odoRL, odoFL)

                        #self._socket2.close()

        except socket.error as msg:
            # if the socket is no longer connected
            print("Error - ErrorConnection - Status Monitoring")
            print("Connect failed. Error Code: " + str(msg[0]) + " - Message: " + msg[1])
    # ------------------------------------------------------------------------------------------------------------------
    def resetIMU(self):
        print("Reset IMU (From propper function)")
        LOCK.acquire()
        request = bytearray([ord('N'), ord('A'), ord('I'), ord('O'), ord('0'), ord('1'), self._commandID, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00])
        try:
            self._socket.send(request)
            LOCK.release()
        except socket.error as msg:
            LOCK.release()
            print("Error - run MoveRobotWithKeyBoard")
            print('Error Code : ' + str(msg[0]) + ' - Message ' + msg[1])
    # ------------------------------------------------------------------------------------------------------------------
    def stop(self):
        self._stop.set()
    # ------------------------------------------------------------------------------------------------------------------
    def stopped(self):
        return self._stop.isSet()
