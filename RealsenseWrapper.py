import threading
import logging            
from sys import stdout, stdin

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

    def setCallback(self, callback):
        self.callback = callback

    def start(self):    #todo : stop
        thread = threading.Thread(target=self.__run)
        thread.start()

    def processSingleFrame(self):
        try:
            frames = self.rsPipeline.wait_for_frames()
            depthFrame = frames.get_depth_frame();

            minDepth = 0

            logging.debug('Process frame #{0}, min depth {1} m'.format(depthFrame.get_frame_number(), minDepth))
            #todo : crop and find closest point
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

    while (stdin.read(1) != 'q'):
        rs.processSingleFrame()


