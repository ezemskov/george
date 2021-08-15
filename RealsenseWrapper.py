import threading
import logging            
import datetime

from sys import stdout, stdin
import cv2 as cv
import numpy as np
import pyrealsense2 as realsense

class RealsenseWrapper:
    LogFilename="realsense_wrapper.log"

    def __init__(self):
        loggerHandlers = [
            logging.StreamHandler(stream=stdout),
            #logging.FileHandler(filename=RealsenseWrapper.LogFilename)
        ] 
        logging.basicConfig(handlers=loggerHandlers, level=logging.DEBUG)

        try:
            self.rsPipeline = realsense.pipeline()
            self.rsProfile = self.rsPipeline.start()
        except Exception as e:
            logging.error(e)

    def setRoi(self, left, right, top, bottom):
        self.left = left
        self.right = right
        self.top = top
        self.bottom = bottom

    def setCallback(self, callback):
        self.callback = callback

    def start(self):    #todo : stop
        thread = threading.Thread(target=self.__run)
        thread.start()

    def processSingleFrame(self):
        try:
            frames = self.rsPipeline.wait_for_frames()
            depthFrame = frames.get_depth_frame();

            depthData = depthFrame.get_data()
            image = np.asarray(bytearray(depthData), dtype="uint16")
            image = cv2.imdecode(image, cv2.IMREAD_GRAYSCALE | IMREAD_ANYDEPTH)

            imageCrop = image[self.top:self.bottom, self.left:self.right]

            minVal, maxVal, minPos, maxPos = cv2.minMaxLoc(imageCrop)
            minDepth = minVal * depthFrame.get_units()

            timestampStr = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            cv2.imwrite("depth_" + timestampStr + ".tiff", image);	
            cv2.imwrite("depth_" + timestampStr + "_crop.tiff", imageCrop);	

            logging.debug('Process frame #{0}, min depth {1} m at {2}'.format(depthFrame.get_frame_number(), minDepth, minPos))

            if (self.callback != None):
                self.callback(minDepth)
        except Exception as e:
            logging.error(e)

    def __run(self):
        while True:
            self.processSingleFrame()
            #sleep ?

def logMinDepth(minDepth):
    logging.debug('Min depth in frame {0}'.format(minDepth))

if __name__ == '__main__':
    rs = RealsenseWrapper()
    rs.setCallback(logMinDepth)
    rs.setRoi(100, 748, 0, 380)

    while (stdin.read(1) != 'q'):
        rs.processSingleFrame()


