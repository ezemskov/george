from ChassisInterface import ChassisInterface
from ROS_ImageSubscriber import ImageSubscriber
from MinDepthRealsenseROS import MinDepthRealsenseROS
from RTOD_ROS import RTOD
import logging
from enum import IntEnum
import rclpy 
import cv2

class Cfg:
    #УЗ-датчик возвращает время от передачи до приема импульса в мксек (?)
    UltrasonicToMeters = 0.5 * 343.0 * 1E-6

    RangeThrMeters = 65.

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

    class RangeId(IntEnum):
        Depth = 1,
        Ultrasonic1 = 2
        Ultrasonic2 = 3
        Ultrasonic3 = 4
        Ultrasonic4 = 5


class CollisionAvoidanceManager:
    def __init__(self):
        self._ranges = {
            Cfg.RangeId.Ultrasonic1 : 0.,
            Cfg.RangeId.Ultrasonic2 : 0.,
            Cfg.RangeId.Ultrasonic3 : 0.,
            Cfg.RangeId.Ultrasonic4 : 0.
        }

        self._chassis = ChassisInterface()
        self._chassis.setUltrasonicCallback(self.updateUltrasonic)

        self._min_depth_calc = MinDepthRealsenseROS()
        #self._min_depth_calc.setRoi(100, 748, 0, 380)
        self._min_depth_calc.setDepthThr(Cfg.RangeThrMeters)

        Cfg.RTODCfg["callback"] = self.updateRTDetection    #NB modifying a global/static dictionary, but who cares
        self._rtod = RTOD(Cfg.RTODCfg)
        self._rosSub = ImageSubscriber()
        #self._rosSub.subscribe('/color/image_raw', self._rtod.ProcessNumpyImage)
        self._rosSub.subscribe('/color/image_raw', CollisionAvoidanceManager.processColorImage)
        self._rosSub.subscribe('/depth/image_rect_raw', self.processDepthImage)

        self._steeringCmdRel = None
        self._speedCmdRel = None

        self._minDepth = -1.
        self._detectionSizePx = {"x" : 0, "y" : 0}

    def processDepthImage(self, image):
        cv2.imwrite(r"latest_depth.tiff", image)
        self._minDepth = self._min_depth_calc.processDepthImage(image)
        self._ranges[Cfg.RangeId.Depth] = self._minDepth #todo : remove _minDepth

    def processColorImage(image):
        cv2.imwrite(r"latest_color.tiff", image)

    #todo : handle multiple detections
    def updateRTDetection(self, startX, startY, endX, endY):
        self._detectionSizePx["x"] = min(endX - startX, 0)
        self._detectionSizePx["y"] = min(endY - startY, 0)

    def updateUltrasonic(self, deviceId, respInt):
        distMeters = Cfg.UltrasonicToMeters * respInt
        self._ranges[deviceId] = distMeters
        logging.debug('Ultrasonic id {0} range {1} ({2} meters)'.format(deviceId, respInt, distMeters))

    def updateCmd(self, motComand, speedCommand):
        #Вычисляем speedCommandRel [-1..1] из направления motComand и модуля скорости speedCommand 
        SpeedCommandMax = 100
        assert (speedCommand >= 0) and (speedCommand <= SpeedCommandMax)
        self._speedCmdRel = speedCommand / SpeedCommandMax

        if motComand == "down":
            self._speedCmdRel *= -1.0
        if motComand == "stop":
            self._speedCmdRel = 0.0
        if motComand == "left":
            self._speedCmdRel = 0.0
        if motComand == "right":
            self._speedCmdRel = 0.0
            
        self._steeringCmdRel = Cfg.SteeringDic.get(motComand)
        assert (self._steeringCmdRel != None)

        #self.limitChassisCmd()
        self._chassis.setSpeed(self._speedCmdRel)
        self._chassis.setSteering(self._steeringCmdRel)
        
    def __LimitCmd(cmd, range):
        if (cmd > 0) and (range > 0):
            if (range < Cfg.RangeSlowMeters) : 
                return min(cmd, 0.1)
            if (range < Cfg.RangeStopMeters) : 
                return 0.
    
    def limitChassisCmd(self):
        minRangeFwd = min(self._ranges[Cfg.RangeId.Ultrasonic1], 
                          self._ranges[Cfg.RangeId.Ultrasonic2]) 
        minRangeAft = min(self._ranges[Cfg.RangeId.Ultrasonic3], 
                          self._ranges[Cfg.RangeId.Ultrasonic4]) 

        self._speedCmdRel = CollisionAvoidanceManager.__LimitCmd(self._speedCmdRel, self._minDepth)

        if (self._speedCmdRel > 0) : 
            self._speedCmdRel = CollisionAvoidanceManager.__LimitCmd(self._speedCmdRel, minRangeFwd)        

        if (self._speedCmdRel < 0) : 
            self._speedCmdRel = -1 * CollisionAvoidanceManager.__LimitCmd(-1 * self._speedCmdRel, minRangeAft)        

        if ((self._detectionSizePx["x"] > Cfg.DetectionSizeMaxPx["x"]) and
            (self._detectionSizePx["y"] > Cfg.DetectionSizeMaxPx["y"])) : 
            self._speedCmdRel = 0

    def GetRange(self, id):
        return self._ranges[id]

    def GetDetectionSize(self):
        return self._detectionSizePx

if __name__ == '__main__':
    rclpy.init()
    coav = CollisionAvoidanceManager()
    rclpy.spin(coav._rosSub)
    rclpy.shutdown()
    
