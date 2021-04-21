from smbus2 import SMBus
import threading
from time import sleep
from enum import IntEnum
#from ctypes import c_int16

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

    SpeedMax = 1000
    SteeringMax = 1000

    def FormatOffset(cmdId, deviceId):
        return ((int(cmdId) << 4) & 0xF0) | (int(deviceId) & 0x0F); 

    #speedRel и steeringRel в дипазоне [-1 .. 1]
    def FormatWheelCmd(speedRel, steeringRel):
        #todo : check parameter ranges
        speedScaled = int(speedRel * Protocol.SpeedMax)
        steeringScaled = int(steeringRel * Protocol.SteeringMax)
        speedBuf = speedScaled.to_bytes(2, byteorder='big')
        steeringBuf = steeringScaled.to_bytes(2, byteorder='big')
        return speedBuf + steeringBuf

    def ParseWheelResp(respBuf):
        if len(respBuf) != 4:
            return tuple()
        
        speedL = int.from_bytes(respBuf[0:2], byteorder='big', signed=True)
        speedR = int.from_bytes(respBuf[2:4], byteorder='big', signed=True)
        return tuple(speedL, speedR)

    def ParseUltrasonicResp(respBuf):
        if len(respBuf) != 2:
            return 0
        
        return int.from_bytes(respBuf[0:2], byteorder='big', signed=True)


class ChassisInterface:
    ResendIntervalSec = 1.0       #Период передачи команд контроллеру
    I2cSlaveAddr = 80             #Адрес I2C slave (на arduino)
    DefaultBusNum = 6             #id i2c-устройства (/dev/i2c-N)

    def __init__(self, busNum = DefaultBusNum):
        self.bus = SMBus(busNum)     #todo : with ?

        self.speed = 0.0
        self.steering = 0.0

        thread = threading.Thread(target=self.__run)
        thread.start()

    #todo : interrupt and join the thread nicely
    #def __del__(self):
        #todo : set event to interrupt thread
        #thread.join();    
    
    def setSpeed(self, speed_) : 
        self.speed = speed_

    def setSteering(self, steering) : 
        self.steering = steering_

    def __run(self):
        while True:
            cmdBytes = Protocol.FormatWheelCmd(self.speed, self.steering)
            self.bus.write_i2c_block_data(ChassisInterface.I2cSlaveAddr, 
                Protocol.FormatOffset(Protocol.CmdId.WheelCmd, Protocol.DeviceId.FrontAxis), cmdBytes)
            self.bus.write_i2c_block_data(ChassisInterface.I2cSlaveAddr, 
                Protocol.FormatOffset(Protocol.CmdId.WheelCmd, Protocol.DeviceId.RearAxis),  cmdBytes)
            
            wheelRespFront = self.bus.read_i2c_block_data(ChassisInterface.I2cSlaveAddr, 
                Protocol.FormatOffset(Protocol.CmdId.WheelResp, Protocol.DeviceId.FrontAxis), 4);
            Protocol.ParseWheelResp(wheelRespFront)
            #todo : callbacks

            wheelRespRear = self.bus.read_i2c_block_data(ChassisInterface.I2cSlaveAddr, 
                Protocol.FormatOffset(Protocol.CmdId.WheelResp, Protocol.DeviceId.RearAxis), 4);
            Protocol.ParseWheelResp(wheelRespRear)

            for devId in range(Ultrasonic1, Ultrasonic4):
                resp = self.bus.read_i2c_block_data(ChassisInterface.I2cSlaveAddr, 
                    Protocol.FormatOffset(UltrasonicQuery, devId), 2);
                Protocol.ParseUltrasonicResp(resp)


            time.sleep(ResendIntervalSec)

if __name__ == '__main__':
    d = ChassisInterface()
