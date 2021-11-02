import os
import socketio
import datetime
import threading 
import time
import sys
from threading import Thread
import re
import traceback
import subprocess
from subprocess import Popen, PIPE, STDOUT 
import select
import signal
import psutil
import gi 
import board
import getch

from CollisionAvoidanceManager import CollisionAvoidanceManager 


gi.require_version('GLib', '2.0')
gi.require_version('GObject', '2.0')
gi.require_version('Gst', '1.0')

from gi.repository import Gst, GObject, GLib

pipeline = None
bus = None
messageGs = None

dit=None
ads = None
PAUSE_START_VIDEO = 0.0
pro=None
inRasbery = True
sio = socketio.Client()
start_time = time.time()
connected = False
last_response = None
ms = 0
video = None
g_speed_command = 0
g_mot_command = "stop"
g_coav = None

def start_stream(video):
    global pro
    global pipeline
    print("before start videotranslation")
    if pro != None: 
        pipeline.set_state(Gst.State.NULL) 
    if(video==2):
        pipeline = Gst.parse_launch("alsasrc device=hw:1,0 ! audioconvert ! audioresample ! webrtcechoprobe ! opusenc bitrate=100000 ! rtpopuspay ! udpsink host=212.109.192.99 port=8002 v4l2src device=/dev/video2 ! image/jpeg,width=1280, height=720,framerate=30/1 ! jpegdec ! omxh264enc control-rate=1  ! h264parse config-interval=3 ! rtph264pay ! udpsink host=ig2u.online port=8004 sync=false ")
        pass
    elif(video==1):
        pipeline = Gst.parse_launch("alsasrc device=hw:1,0 ! audioconvert ! audioresample ! webrtcechoprobe ! opusenc bitrate=100000 ! rtpopuspay ! udpsink host=212.109.192.99 port=8002 v4l2src device=/dev/video3 ! image/jpeg,width=1280, height=720,framerate=30/1 ! jpegdec ! omxh264enc control-rate=1  ! h264parse config-interval=3 ! rtph264pay ! udpsink host=ig2u.online port=8004 sync=false ")
        pass       
    elif(video==0):
        pipeline.set_state(Gst.State.NULL) 
        pass
    elif(video==3):
        pipeline = Gst.parse_launch("alsasrc device=hw:1,0 ! audioconvert ! audioresample ! webrtcechoprobe ! opusenc bitrate=100000 ! rtpopuspay ! udpsink host=212.109.192.99 port=8002 v4l2src device=/dev/video1 ! image/jpeg,width=1280, height=720,framerate=30/1 ! jpegdec ! omxh264enc control-rate=1  ! h264parse config-interval=3 ! rtph264pay ! udpsink host=ig2u.online port=8004 sync=false")
        pass
    elif(video==4):
        pipeline = Gst.parse_launch("alsasrc device=hw:1,0 ! audioconvert ! audioresample ! webrtcechoprobe ! opusenc bitrate=100000 ! rtpopuspay ! udpsink host=212.109.192.99 port=8002 v4l2src device=/dev/video0 ! image/jpeg,width=1280, height=720,framerate=30/1 ! jpegdec ! omxh264enc control-rate=1  ! h264parse config-interval=3 ! rtph264pay ! udpsink host=ig2u.online port=8004 sync=false")
        pass
#    elif(video==5):
 #       pipeline = Gst.parse_launch("alsasrc device=hw:2,0 ! audioconvert ! audioresample ! webrtcdsp noise-suppression-level=3 voice-detection=true voice-detection-likelihood=3 echo-suppression-level=2 ! webrtcechoprobe ! opusenc bitrate=100000 ! rtpopuspay ! udpsink host=62.109.16.164 port=8002")
  #      pass
    elif(video==6):
        pipeline = Gst.parse_launch("alsasrc device=hw:1,0 ! audioconvert ! audioresample ! webrtcechoprobe ! opusenc bitrate=100000 ! rtpopuspay ! udpsink host=212.109.192.99 port=8002 v4l2src device=/dev/video0 ! image/jpeg,width=1280, height=720,framerate=30/1 ! jpegdec ! omxh264enc control-rate=1  ! h264parse config-interval=3 ! rtph264pay ! udpsink host=ig2u.online port=8004 sync=false")
        pass
    # start playing
    pipeline.set_state(Gst.State.PLAYING)
    pro = 1
    # wait until EOS or error
    bus = pipeline.get_bus()
    msg = bus.timed_pop_filtered(
        Gst.CLOCK_TIME_NONE,
        Gst.MessageType.ERROR | Gst.MessageType.EOS
        )
    # free resources
    pipeline.set_state(Gst.State.NULL)
    print("after start videotranslation")

#перевод углов в команды, а - углы, b - сигналы, s - угол (вызов из test_broadcast_message)
def map_range(a,b,s): 
    (a1,a2),(b1,b2)=a,b
    return b1 + ((s-a1)*(b2-b1))/(a2-a1)


@sio.event(namespace='/test') #действия при запуске
def connect():
    global dit
    global ads
    global connected
#    global start_time
    global ms
# initialize GStreamer
    Gst.init(sys.argv[1:])
    global pipeline
    pipeline = Gst.Pipeline()
    print(pipeline)
    last_response = datetime.datetime.utcnow()
    connected = True
    print('connection established')

def handle_motion_command_i2c(motComand, speedCommand):
    global g_coav

    print("handle_motion_command_i2c {0} {1}\r\n".format(motComand, speedCommand))
    g_coav.updateCmd(motComand, speedCommand)

#ОБРАБОТКА управляющих сообщений от сервера (запускается после получения сообщения от сервера)
@sio.on('my_responseIO', namespace='/test')
def test_broadcast_message(data):
    print('message received with ', data)
    global last_response
    global dit
    global video
    last_response = datetime.datetime.utcnow()
    params = data.split("-")                                         # разбиваем строку сообщения от сервера
    if 1==1:
     mode = params[0]
     if mode == "ser":
    #===== запускаем и глушим видео в засисимости от цифры в сообщении от сервера
         if len(params)>3:
          video=int(params[3])
          if video!=None:
           print('video = ', video)
           start_stream(video)
           
    if mode == "mot":
     try:
      if inRasbery:
       g_mot_command = params[1]
       g_speed_command = int(params[2])
       handle_motion_command_i2c(g_mot_command, g_speed_command)
       
     except Exception as e:
      pass

@sio.event(namespace='/test')
def disconnect():
    print('disconnected from server')
    global dit
    global connected
    global video 
    connected = False
    pipeline.set_state(Gst.State.NULL) 
    if dit!=None:
     dit.set_servo_pulsewidth(18,0)                      #отключаем питание моторов
     #dit.set_servo_pulsewidth(27,0)
     dit.stop()                                          #создаем объект для работы с моторами
     dit=None
     print('dit stop 98', dit)
    if video!=0:
     stop_video()

def ping_pongStart():
    global start_time
    start_time = datetime.datetime.utcnow()
    sio.emit('my_pingR', {u'data': u''}, namespace='/test')
    
#ответ на событие my_pongR с сервера - расчет задержки
@sio.on('my_pongR', namespace='/test')
def latencyR(data):
    global start_time
#    global ping_pong_time
    global ms
    ping_pong_time = (str)((datetime.datetime.utcnow() - start_time)/2)
 #   ping_pong_time = (datetime.datetime.utcnow() - start_time)/2
    secondS = ping_pong_time.split(':')
    ms = round(float(secondS[2])*1000, 2)
#    print('задержка = ', ms)
    return ms

#передача данных на сервер
def ping_pongR(data):
    global start_time
    start_time = datetime.datetime.utcnow()
    sio.emit('my_pingR', {u'data': data}, namespace='/test')

#передача данных на сервер
def ping_pongV(data):
    sio.emit('my_pingV', {u'data': data}, namespace='/test')

def main_remote():
    while True:
        try:
            sio.connect('https://evgenium.fvds.ru:5000') #адрес для коннекта
            sio.wait()
        except Exception as e:
            print(e)

def handle_keypress(key):
    global g_speed_command
    global g_mot_command

    CmdDic = {
        'w' : "top",
        's' : "down",
        'a' : "left",
        'd' : "right",
        ' ' : "stop"
    }

    SpeedDic = {
        '+' : lambda cmdVal : min(cmdVal  + 10, 100),
        '-' : lambda cmdVal : max(cmdVal  - 10, 0) 
    }

    speedFunc = SpeedDic.get(key)
    if speedFunc != None:
        g_speed_command = speedFunc(g_speed_command)

    motCmd = CmdDic.get(key)
    if motCmd != None:
        g_mot_command = motCmd

    if (speedFunc != None) or (motCmd != None):
       handle_motion_command_i2c(g_mot_command, g_speed_command)

def main_console():
    global g_coav
    while True:          
        try:                 
           key = getch.getch()         
           if key == 'q':
              del g_coav
              sys.exit()
           if key != -1:
              handle_keypress(key)
        except Exception as e:
           print ("Exception '" + str(e) + "'\r\n")

def main_sleep():
    global g_coav
    rclpy.init()

    g_coav = CollisionAvoidanceManager()

    rclpy.spin(coav._rosSub)
    rclpy.shutdown()

#main_sleep()
main_console()
#main_remote()
