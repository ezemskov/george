# USAGE
# python real_time_object_detection.py --prototxt MobileNetSSD_deploy.prototxt.txt --model MobileNetSSD_deploy.caffemodel --topic /color/image_raw

import numpy as np
import argparse
import imutils
import time
import cv2


from ROS_ImageSubscriber import ImageSubscriberWrapper

class RTOD:
  # initialize the list of class labels MobileNet SSD was trained to
  # detect, then generate a set of bounding box colors for each class
  CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
    "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
    "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
    "sofa", "train", "tvmonitor"]

  COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

  def __init__(self, args_):
    self.args = args_

    # load our serialized model from disk
    print("[INFO] loading model...")
    self.net = cv2.dnn.readNetFromCaffe(self.args["prototxt"], self.args["model"])

  def NumpyImageCallback(self, frame):
    frameResized = imutils.resize(frame, width=400)

    # grab the frame dimensions and convert it to a blob
    (h, w) = frameResized.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(frameResized, (300, 300)),
        0.007843, (300, 300), 127.5)

    # pass the blob through the network and obtain the detections and
    # predictions
    self.net.setInput(blob)
    detections = self.net.forward()

    # loop over the detections
    for i in np.arange(0, detections.shape[2]):
        # extract the confidence (i.e., probability) associated with
        # the prediction
        confidence = detections[0, 0, i, 2]

        # filter out weak detections by ensuring the `confidence` is
        # greater than the minimum confidence
        if confidence > args["confidence"]:
            # extract the index of the class label from the
            # `detections`, then compute the (x, y)-coordinates of
            # the bounding box for the object
            idx = int(detections[0, 0, i, 1])
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")
            
            # draw the prediction on the frame
            label = "{}: {:.2f}%".format(RTOD.CLASSES[idx], confidence * 100)
            cv2.rectangle(frameResized, (startX, startY), (endX, endY),
                RTOD.COLORS[idx], 2)
            y = startY - 15 if startY - 15 > 15 else startY + 15
            print("#{} label '{}' rect [{} {} {} {}]".format(i, label, startY, endY, startX, endX))
            cv2.putText(frameResized, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, RTOD.COLORS[idx], 2)

    #cv2.imwrite('rtod_result.png', frameResized)
    #cv2.imshow("Frame", frame)
    #cv2.imshow("Frame resized", frameResized)
    cv2.waitKey(1)


# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-p", "--prototxt", required=True,
    help="path to Caffe 'deploy' prototxt file")
ap.add_argument("-m", "--model", required=True,
    help="path to Caffe pre-trained model")
ap.add_argument("-c", "--confidence", type=float, default=0.2,
    help="minimum probability to filter weak detections")
ap.add_argument("-t", "--topic", type=str, required=True,
    help="ROS topic name to subscribe on")
args = vars(ap.parse_args())


rtod = RTOD(args) 

ros_subscriber = ImageSubscriberWrapper(args["topic"], rtod.NumpyImageCallback)

# do a bit of cleanup
cv2.destroyAllWindows()
