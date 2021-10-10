from ChassisInterface import ChassisInterface, Protocol
from ROS_ImageSubscriber import ImageSubscriberWrapper
from MinDepthRealsenseROS import MinDepthRealsenseROS
from RTOD_ROS import RTOD

class Cfg:
    #УЗ-датчик возвращает время от передачи до приема импульса в мксек (?)
    UltrasonicToMeters = 0.5 * 343.0 * 1E-6

    RangeSlowMeters = 1.0
    RangeStopMeters = 0.4

    DetectionSizeMaxPx = {
        "x" : 100,
        "y" : 200
    }    

    SteeringDic = {
        "top" : 0.0,
        "down" : 0.0,
        "left" : -1.0,
        "right" : 1.0,
        "stop" : 0.0
    }

    RTODCfg = {
        "prototxt"   : "MobileNetSSD_deploy.prototxt.txt",
        "model"      : "MobileNetSSD_deploy.caffemodel",
        "confidence" : 0.2,
        "class"      : "person"
    }

class CollisionAvoidanceManager:
    def __init__(self):
        self._ultrasonicRanges = {
            Protocol.DeviceId.Ultrasonic1 : 0.,
            Protocol.DeviceId.Ultrasonic2 : 0.,
            Protocol.DeviceId.Ultrasonic3 : 0.,
            Protocol.DeviceId.Ultrasonic3 : 0.
        }

        self._chassis = ChassisInterface()
        self._chassis.setUltrasonicCallback(self.updateUltrasonic)

        self._min_depth_calc = MinDepthRealsenseROS()
        self._min_depth_calc.setRoi(100, 748, 0, 380)

        Cfg.RTODCfg["callback"] = self.updateRTDetection    #NB modifying a global/static dictionary, but who cares
        self._rtod = RTOD(Cfg.RTODCfg)
        self._rosSub = ImageSubscriberWrapper()
        self._rosSub.subscribe('/color/image_raw', self._rtod.ProcessNumpyImage)
        self._rosSub.subscribe('/depth/image_rect_raw', self.processDepthImage)

        self._steeringCmdRel = None
        self._speedCmdRel = None

        self._minDepth = -1.
        self._detectionSizePx = {"x" : 0, "y" : 0}

    def processDepthImage(self, depthImage):
        self._minDepth = self._min_depth_calc.processDepthImage(depthImage)

    #todo : handle multiple detections
    def updateRTDetection(self, startX, startY, endX, endY):
        self._detectionSizePx["x"] = min(endX - startX, 0)
        self._detectionSizePx["y"] = min(endY - startY, 0)

    def updateUltrasonic(self, deviceId, respInt):
        distMeters = Cfg.UltrasonicToMeters * respInt
        self._ultrasonicRanges[deviceId] = distMeters

    def updateCmd(self, motComand, speedCommand):
        #Вычисляем speedCommandRel [-1..1] из направления motComand и модуля скорости speedCommand 
        SpeedCommandMax = 100
        assert (speedCommand >= 0) and (speedCommand <= SpeedCommandMax)
        self._speedCmdRel = speedCommand / SpeedCommandMax

        if motComand == "down":
            self._speedCmdRel *= -1.0
        if motComand == "stop":
            self._speedCmdRel = 0.0

        self._steeringCmdRel = Cfg.SteeringDic.get(motComand)
        assert (self._steeringCmdRel != None)

        self.limitChassisCmd()
        self._chassis.setSpeed(self._speedCmdRel)
        self._chassis.setSteering(self._steeringCmdRel)
        
    def __LimitCmd(cmd, range):
        if (cmd > 0) and (range > 0):
            if (range < Cfg.RangeSlowMeters) : 
                return min(cmd, 0.1)
            if (range < Cfg.RangeStopMeters) : 
                return 0.
    
    def limitChassisCmd(self):
        minRangeFwd = min(self._ultrasonicRanges[Protocol.DeviceId.Ultrasonic1], 
                          self._ultrasonicRanges[Protocol.DeviceId.Ultrasonic2]) 
        minRangeAft = min(self._ultrasonicRanges[Protocol.DeviceId.Ultrasonic3], 
                          self._ultrasonicRanges[Protocol.DeviceId.Ultrasonic4]) 

        self._speedCmdRel = CollisionAvoidanceManager.__LimitCmd(self._speedCmdRel, self._minDepth)

        if (self._speedCmdRel > 0) : 
            self._speedCmdRel = CollisionAvoidanceManager.__LimitCmd(self._speedCmdRel, minRangeFwd)        

        if (self._speedCmdRel < 0) : 
            self._speedCmdRel = -1 * CollisionAvoidanceManager.__LimitCmd(-1 * self._speedCmdRel, minRangeAft)        

        if ((self._detectionSizePx["x"] > Cfg.DetectionSizeMaxPx["x"]) and
            (self._detectionSizePx["y"] > Cfg.DetectionSizeMaxPx["y"])) : 
            self._speedCmdRel = 0

if __name__ == '__main__':
    coav = CollisionAvoidanceManager()