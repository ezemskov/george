import curses
import os
import socketio
import datetime
import time
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
import board
from ChassisInterface import ChassisInterface
from ROS_ImageSubscriber import ImageSubscriberWrapper
from MinDepthRealsenseROS import MinDepthRealsenseROS

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
g_chassis = None
g_ros_sub = None
g_min_depth_calc = None
g_minDepth = -1

#убиваем процесс по имени (вызов из stop_video)
def kill_name():
    for proc in psutil.process_iter():
      if proc.name() == 'gst-launch-1.0':
         proc.kill()
    
#старт стереовидео (вызов из test_broadcast_message)
def start_videoS():
    print("before start videotranslation")
    global pro
    if pro != None:
        stop_video()
#        os.killpg(os.getpgid(pro.pid), signal.SIGTERM)  # Send the signal to all the process groups
        time.sleep(1)
    print('стерео: ', pro)
    pro = subprocess.Popen(['/home/pi/to_janusIg2u.sh'], shell=True, preexec_fn=os.setsid) 
    print("after start videotranslation")

#старт моновидео (вызов из test_broadcast_message)
def start_video():
    global pro
    print("before start videotranslation")
    if pro != None:
       stop_video()
#       os.killpg(os.getpgid(pro.pid), signal.SIGTERM)  # Send the signal to all the process groups
       time.sleep(1)
    print('моно: ', pro)
    pro = subprocess.Popen(['/home/pi/to_janusIg2u_crop.sh'], shell=True, preexec_fn=os.setsid) 
#    return str(pro.communicate()[0])
    print("after start videotranslation")

#старт стереовидео со звуком (вызов из test_broadcast_message)
def start_videoSZ():
    print("before start videotranslation")
    global pro
    if pro != None:
        stop_video()
#        os.killpg(os.getpgid(pro.pid), signal.SIGTERM)  # Send the signal to all the process groups
        time.sleep(1)
    print('стерео: ', pro)
    pro = subprocess.Popen(['/home/pi/to_janusIg2u_A.sh'], shell=True, preexec_fn=os.setsid) 
    print("after start videotranslation")

#старт моновидео со звуком(вызов из test_broadcast_message)
def start_videoZ():
    global pro
    print("before start videotranslation")
    if pro != None:
       stop_video()
#       os.killpg(os.getpgid(pro.pid), signal.SIGTERM)  # Send the signal to all the process groups
       time.sleep(1)
    print('моно: ', pro)
    pro = subprocess.Popen(['/home/pi/to_janusIg2u_crop_A.sh'], shell=True, preexec_fn=os.setsid) 
#    return str(pro.communicate()[0])
    print("after start videotranslation")
    

#старт камеры заднего вида (вызов из test_broadcast_message)
def start_videoZad():
    global pro
    print("before start videotranslation")
    if pro != None:
       stop_video()
#       os.killpg(os.getpgid(pro.pid), signal.SIGTERM)  # Send the signal to all the process groups
       time.sleep(1)
    print('моно: ', pro)
    pro = subprocess.Popen(['/home/pi/to_janusIg2u_zad.sh'], shell=True, preexec_fn=os.setsid) 
#    return str(pro.communicate()[0])
    print("after start videotranslation") 

#старт трансляции звука(вызов из test_broadcast_message)   
def start_noise():
    global pro
    print("before start videotranslation")
    if pro != None:
       stop_video()
#       os.killpg(os.getpgid(pro.pid), signal.SIGTERM)  # Send the signal to all the process groups
       time.sleep(1)
    print('моно: ', pro)
    pro = subprocess.Popen(['/home/pi/audIg2u.sh'], shell=True, preexec_fn=os.setsid) 
#    return str(pro.communicate()[0])
    print("after start videotranslation")

#останавливаем стрим видео с робота на сервер (вызов из test_broadcast_message)
def stop_video():
    global pro
    print("before stop videotranslation")
    try:
      print(pro.pid)
      kill_name()
      time.sleep(3)  
      pro=None
      print('kill', pro)
    except Exception as e:
        pass

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
    last_response = datetime.datetime.utcnow()
    connected = True
    print('connection established')

def processDepthImage(depthImage):
    g_minDepth = g_min_depth_calc.processDepthImage(depthImage)

def handle_motion_command_i2c(motComand, speedCommand):
    global g_chassis

    if g_chassis == None:
        g_chassis = ChassisInterface()

    if g_min_depth_calc == None:
        g_min_depth_calc = MinDepthRealsenseROS()

    if g_ros_sub == None:
        g_ros_sub = ImageSubscriberWrapper('/depth/image_rect_raw')
        g_ros_sub.setRoi(100, 748, 0, 380)
        g_ros_sub.setCallback(processDepthImage)

    print("handle_motion_command_i2c {0} {1}\r\n".format(motComand, speedCommand))

    #Вычисляем speedCommandRel [-1..1] из направления motComand и модуля скорости speedCommand 
    SpeedCommandMax = 100
    assert (speedCommand >= 0) and (speedCommand <= SpeedCommandMax)
    speedCmdRel = speedCommand / SpeedCommandMax

    if motComand == "down":
        speedCmdRel *= -1.0
    if motComand == "stop":
        speedCmdRel = 0.0

    SteeringDic = {
        "top" : 0.0,
        "down" : 0.0,
        "left" : -1.0,
        "right" : 1.0,
        "stop" : 0.0
    }

    steeringCmdRel = SteeringDic.get(motComand)
    assert (steeringCmdRel != None)

    g_chassis.setSpeed(speedCmdRel)
    g_chassis.setSteering(steeringCmdRel)

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
           if(video==2):
               threading.Timer(PAUSE_START_VIDEO, start_videoS).start()
               start_videoS()           
               pass
           elif(video==1):
               threading.Timer(PAUSE_START_VIDEO, start_video).start()
               start_video()
               pass       
           elif(video==0):
               threading.Timer(PAUSE_START_VIDEO, stop_video).start()
               stop_video()
               pass
           elif(video==3):
               threading.Timer(PAUSE_START_VIDEO, start_videoZ).start()
               start_videoZ()
               pass
           elif(video==4):
               threading.Timer(PAUSE_START_VIDEO, start_videoSZ).start()
               start_videoSZ()
               pass
           elif(video==5):
               threading.Timer(PAUSE_START_VIDEO, start_noise).start()
               start_noise()
           elif(video==6):
               threading.Timer(PAUSE_START_VIDEO, start_videoZad).start()
               start_videoZad()
               pass               

    if mode == "mot":
     try:
      if inRasbery:
       g_mot_command = params[1]
       g_speed_command = int(params[2])
       #handle_motion_command_gpio_pwm(g_mot_command, g_speed_command)
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
        curses.KEY_UP : "top",
        curses.KEY_DOWN : "down",
        curses.KEY_LEFT : "left",
        curses.KEY_RIGHT : "right",
        ord(' ') : "stop"
    }

    SpeedDic = {
        ord('+') : lambda cmdVal : min(cmdVal  + 10, 100),
        ord('-') : lambda cmdVal : max(cmdVal  - 10, 0) 
    }

    speedFunc = SpeedDic.get(key)
    if speedFunc != None:
        g_speed_command = speedFunc(g_speed_command)

    motCmd = CmdDic.get(key)
    if motCmd != None:
        g_mot_command = motCmd

    if (g_minDepth > 0):
        if (g_minDepth < 1.) : 
            g_speed_command = min(g_speed_command, 10)
        if (g_minDepth < 0.4) : 
            g_speed_command = 0

    #handle_motion_command_gpio_pwm(g_mot_command, g_speed_command)
    handle_motion_command_i2c(g_mot_command, g_speed_command)

def main_console(win):
    win.timeout(10) #msec

    while True:          
        try:                 
           key = win.getch()         
           if key == ord('q'):
              quit()           
           if key != -1:
              handle_keypress(key)
        except Exception as e:
           print ("Exception '" + str(e) + "'\r\n")

#curses.wrapper(main_console)
main_remote()
