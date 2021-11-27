import logging            

from sys import stdout
import cv2
import numpy

class Cfg:
    LogFilename="realsense.log"
    DepthUnitMeters = 0.001

class MinDepthRealsenseROS:
    def __init__(self):
        loggerHandlers = [
            logging.StreamHandler(stream=stdout),
            logging.FileHandler(filename=Cfg.LogFilename)
        ] 
        logging.basicConfig(handlers=loggerHandlers, level=logging.DEBUG)

        self.left = -1
        self.right = -1
        self.top = -1
        self.bottom = -1


    def setRoi(self, left, right, top, bottom):
        self.left = left
        self.right = right
        self.top = top
        self.bottom = bottom

    def setDepthThr(self, val):
        self.depthThr = val

    def processDepthImage(self, numpyImage):
        try:

            numpyCrop = numpyImage
            useRoi = (self.left >= 0) and (self.right >= 0) and (self.top >= 0) and (self.bottom >= 0) 
            if useRoi : 
                numpyCrop = numpyImage[self.top:self.bottom, self.left:self.right]
            depthThrPixVal = int(self.depthThr / Cfg.DepthUnitMeters)

            numpyThresholded = numpy.where(numpyCrop == 0, depthThrPixVal, numpyCrop)
            cv2.imwrite(r"latest_thr.tiff", numpyThresholded)

            minVal, maxVal, minPos, maxPos = cv2.minMaxLoc(numpyThresholded)
            minDepth = minVal * Cfg.DepthUnitMeters

            logging.debug('Process frame : min depth {0} m at {1}'.format(minDepth, minPos))
            return minDepth
        except Exception as e:
            logging.error("MinDepthRealsenseROS : " + str(e))
            return -1
