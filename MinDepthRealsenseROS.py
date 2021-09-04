import logging            

from sys import stdout
import cv2

class MinDepthRealsenseROS:
    LogFilename="realsense.log"
    DepthUnitMeters = 0.001

    def __init__(self):
        loggerHandlers = [
            logging.StreamHandler(stream=stdout),
            #logging.FileHandler(filename=MinDepthRealsenseROS.LogFilename)
        ] 
        logging.basicConfig(handlers=loggerHandlers, level=logging.DEBUG)

    def setRoi(self, left, right, top, bottom):
        self.left = left
        self.right = right
        self.top = top
        self.bottom = bottom

    def processDepthImage(self, numpyImage):
        try:
            numpyCrop = numpyImage[self.top:self.bottom, self.left:self.right]
            minVal, maxVal, minPos, maxPos = cv2.minMaxLoc(numpyCrop)
            minDepth = minVal * MinDepthRealsenseROS.DepthUnitMeters

            logging.debug('Process frame : min depth {1} m at {2}'.format(minDepth, minPos))
            return minDepth
        except Exception as e:
            logging.error(e)
            return -1
