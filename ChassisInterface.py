from smbus2 import SMBus
import threading
from time import sleep
from enum import IntEnum
import logging            
from sys import stdout

class Protocol:
    class CmdId(IntEnum):
        WheelCmd = 1
        WheelResp = 2
        UltrasonicQuery = 3

    class DeviceId(IntEnum):
        FrontAxis = 0
        RearAxis = 1
        Ultrasonic1 = 2
        Ultrasonic2 = 3
        Ultrasonic3 = 4
        Ultrasonic4 = 5

    SpeedMax = 200
    SteeringMax = 500

    def FormatOffset(cmdId, deviceId):
        return ((int(cmdId) << 4) & 0xF0) | (int(deviceId) & 0x0F); 

    #speedRel и steeringRel в дипазоне [-1 .. 1]
    def FormatWheelCmd(speedRel, steeringRel):
        #todo : check parameter ranges
        speedScaled = int(speedRel * Protocol.SpeedMax)
        steeringScaled = int(steeringRel * Protocol.SteeringMax)
        speedBuf = speedScaled.to_bytes(2, byteorder='big', signed=True)
        steeringBuf = steeringScaled.to_bytes(2, byteorder='big', signed=True)
        checksumBuf = []

        for x, y in zip(speedBuf, steeringBuf):
            xorByte = x ^ y
            checksumBuf.append(xorByte)

        return speedBuf + steeringBuf + bytes(checksumBuf)

    def ParseWheelResp(respBuf):
        if len(respBuf) < 4:
            return None
        
        speedL = int.from_bytes(respBuf[0:2], byteorder='big', signed=True)
        speedR = int.from_bytes(respBuf[2:4], byteorder='big', signed=True)
        return (speedL, speedR)

    def ParseUltrasonicResp(respBuf):
        if len(respBuf) < 2:
            return None
        
        return int.from_bytes(respBuf[0:2], byteorder='big', signed=True)

class SMBusStub:
    def write_i2c_block_data(self, address, offset, data):
        pass
    def read_i2c_block_data(self, address, offset, dataSize):
        return b'\xAB\xCD\xEF\x12'

class ChassisInterface:
    LogFilename="chassis_interface.log"
    LogRecordFormat="%(asctime)s %(levelname)s %(message)s"
    ResendIntervalSec = 1.0       #Период передачи команд контроллеру
    InterCmdRotatePauseSeс = 0.1  #Интервал между I2C-транзакциями для поворотов
    InterCmdPauseSeс = 0.05       #Интервал между I2C-транзакциями
    I2cSlaveAddr = 80             #Адрес I2C slave (на arduino)
    DefaultBusNum = 1             #id i2c-устройства (/dev/i2c-N)

    def __init__(self, busNum = DefaultBusNum):

        loggerHandlers = [
            logging.StreamHandler(stream=stdout),
            logging.FileHandler(filename=ChassisInterface.LogFilename)
        ] 
        logging.basicConfig(handlers=loggerHandlers, level=logging.INFO, format=ChassisInterface.LogRecordFormat)

        self.bus = SMBus(busNum)
        #self.bus = SMBusStub()

        self.speed = 0.0
        self.speedChanged = True
        self.steering = 0.0
        self.steeringChanged = True
        self.wheelCallback = None
        self.ultrasonicCallback = None

        self.thread_stop_event = threading.Event()
        self.thread = threading.Thread(target=self.__run, daemon=True)
        self.thread.start()

    def __del__(self):
        self.thread_stop_event.set()
        self.thread.join();    
    
    def setWheelCallback(self, callback):
        self.wheelCallback = callback

    def setUltrasonicCallback(self, callback):
        self.ultrasonicCallback = callback

    def setSpeed(self, speed_) : 
        self.speedChanged = (self.speed != speed_)
        self.speed = speed_

    def setSteering(self, steering_) : 
        self.steeringChanged = (self.steering != steering_)
        self.steering = steering_

    def __sendFormattedWheelCmd(self, offs, cmdBytes):        
        logging.debug('Send to offset {0}: [0x{1}]'.format(hex(offs), cmdBytes.hex()))
        try:
            self.bus.write_i2c_block_data(ChassisInterface.I2cSlaveAddr, offs, cmdBytes)
        except OSError as exc:
            logging.error('I2C error : '.format(exc))

    def __sendWheelCmd(self, deviceId):
        if (deviceId == Protocol.DeviceId.FrontAxis) : 
            steering = self.steering
        else:
            steering = -1 * self.steering

        offs = Protocol.FormatOffset(Protocol.CmdId.WheelCmd, deviceId)
        cmdBytes = Protocol.FormatWheelCmd(self.speed, steering)
        self.__sendFormattedWheelCmd(offs, cmdBytes)

    def __sendWheelCmdStop(self, deviceId):
        offs = Protocol.FormatOffset(Protocol.CmdId.WheelCmd, deviceId)
        cmdBytes = Protocol.FormatWheelCmd(0, 0)
        self.__sendFormattedWheelCmd(offs, cmdBytes)

    def __receiveWheelResponse(self, deviceId):
        offs = Protocol.FormatOffset(Protocol.CmdId.WheelResp, deviceId)
        try:
            respList = self.bus.read_i2c_block_data(ChassisInterface.I2cSlaveAddr, offs, 4)
            respBytes = bytes(respList)
            logging.debug('Received from offset {0}: [0x{1}]'.format(hex(offs), respBytes.hex()))

            respTuple = Protocol.ParseWheelResp(respBytes)
            if (self.wheelCallback != None and respTuple != None) : 
                self.wheelCallback(deviceId, respTuple)
        except OSError as exc:
            logging.error('I2C error : '.format(exc))

    def __receiveUltrasonicResponse(self, deviceId):
        offs = Protocol.FormatOffset(Protocol.CmdId.UltrasonicQuery, deviceId)
        try:
            respList = self.bus.read_i2c_block_data(ChassisInterface.I2cSlaveAddr, offs, 2)
            respBytes = bytes(respList)
            logging.debug('Received from offset {0}: [0x{1}]'.format(hex(offs), respBytes.hex()))
            respInt = Protocol.ParseUltrasonicResp(respBytes)
            if (self.ultrasonicCallback != None and respInt != None) : 
                if (respInt > 0):
                    logging.debug('respInt ', respInt)
                    self.ultrasonicCallback(deviceId, respInt)
        except OSError as exc:
            logging.error('I2C error : '.format(exc))

    def __run(self):
        while (self.thread_stop_event.is_set() == False):
             if (self.speedChanged or self.steeringChanged):
             ### повороты с заданной длительностью
                if (self.steering != 0.0):
                    #logging.debug('steering ', self.steering)
                    self.__sendWheelCmd(Protocol.DeviceId.FrontAxis)
                    sleep(ChassisInterface.InterCmdPauseSeс)   
                    self.__sendWheelCmd(Protocol.DeviceId.RearAxis)
                    sleep(2*ChassisInterface.InterCmdRotatePauseSeс)
                    
                    self.__sendWheelCmdStop(Protocol.DeviceId.RearAxis)
                    sleep(ChassisInterface.InterCmdPauseSeс)
                    self.__sendWheelCmdStop(Protocol.DeviceId.RearAxis)
                    self.steering = 0.0
                else:    
                    self.__sendWheelCmd(Protocol.DeviceId.FrontAxis)
                    sleep(ChassisInterface.InterCmdPauseSeс)   
                    self.__sendWheelCmd(Protocol.DeviceId.RearAxis)
                    sleep(2*ChassisInterface.InterCmdPauseSeс)   
            
                self.speedChanged=False
                self.steeringChanged=False
#                self.__receiveWheelResponse(Protocol.DeviceId.FrontAxis)
#                sleep(ChassisInterface.InterCmdPauseSeс)   
#                self.__receiveWheelResponse(Protocol.DeviceId.RearAxis)
#                sleep(ChassisInterface.InterCmdPauseSeс)   

             for devId in range(Protocol.DeviceId.Ultrasonic1, Protocol.DeviceId.Ultrasonic4+1):
                    self.__receiveUltrasonicResponse(devId)
                    sleep(ChassisInterface.InterCmdPauseSeс)   
          
#            sleep(ChassisInterface.ResendIntervalSec)

def logWheelResp(deviceId, respTuple):
    logging.debug('Received wheel response from id {0} : L {1} R {2}'.format(deviceId, respTuple[0], respTuple[1]))

def logUltrasonicResp(deviceId, respVal):
    logging.debug('Received ultrasonic response from id {0} : {1} '.format(deviceId, respVal))

if __name__ == '__main__':
    
    d = ChassisInterface()
    d.setWheelCallback(logWheelResp)
    d.setUltrasonicCallback(logUltrasonicResp)


