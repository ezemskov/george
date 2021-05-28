import os
import socketio
import datetime
import time
import pigpio
import threading 
import time
import sys
from threading import Thread
import re
import traceback
import subprocess
import logging
from subprocess import Popen, PIPE, STDOUT 
import select
import signal
import psutil
import RPi.GPIO as GPIO
import board

dit=None
pwm1 = None
pwm2 = None
ads = None
PAUSE_START_VIDEO = 0.0         # пауза перед стартом видео
pro=None
DIF_ANGLE = -1
inRasbery = True
sio = socketio.Client()
mob_range = (0,180) #диапазон углов
robo_range_18 = (600, 2400)
#robo_range_27 = (600, 2400) #диапазон сигналов на пин27
#robo_range_17 = (600, 2400) #диапазон сигналов на пин17
start_time = time.time()
connected = False
last_response = None
ms = 0
video = None
GPIO.setmode(GPIO.BCM)
GPIO.setup(22, GPIO.OUT)
GPIO.setup(27, GPIO.OUT)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(25, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)

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
    global pwm1
    global pwm2
    global ads
    global connected
#    global start_time
    global ms
    last_response = datetime.datetime.utcnow()
    connected = True
    dit=pigpio.pi() #создаем объект для работы с моторами
    if pwm1 == None:
         pwm1 = GPIO.PWM(22, 100)
         pwm1.start(0)
    if pwm2 == None:
         pwm2 = GPIO.PWM(25, 100)
         pwm2.start(0)
         print('motors on!')
    print('connection established')
    try:
        while connected:   #проверяем были ли отправлены команды с клиента в течении 10 секунд, если нет - глушим момторы
            sio.sleep(1)
            ping_pongR(ms)
            delta =  datetime.datetime.utcnow() - last_response
#            print("last active is : " + str(last_response))
            if delta > datetime.timedelta(seconds=10) and dit!=None:
                print("delta is : " + str(delta) + " DIT MUST BE DESTROYED!")
                dit.set_servo_pulsewidth(18,0)                      #отключаем писание моторов
                #dit.set_servo_pulsewidth(27,0)
                dit.stop()                                          #stop объект для работы с моторами
                dit=None
                print('dit stop 98', dit)
#            else:
#                print("delta is : " + str(delta) + " DIT CAN BE ACTIVE!")
    except KeyboardInterrupt:
        print('interrupted!')


#ОБРАБОТКА управляющих сообщений от сервера (запускается после получения сообщения от сервера)
@sio.on('my_responseIO', namespace='/test')
def test_broadcast_message(data):
    print('message received with ', data)
    global last_response
    global dit
    global pwm1
    global pwm2

    global video
    last_response = datetime.datetime.utcnow()
    params = data.split("-")                                         # разбиваем строку сообщения от сервера
    if dit==None:
     dit=pigpio.pi() #создаем объект для работы с моторами
     print('motors on!')
    if 1==1:
     mode = params[0]
     if mode == "ser":

         if dit==None:
            dit=pigpio.pi() #создаем объект для работы с моторами
            print('motors on!')

         x0=int(params[1])
         y0=int(params[2])

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
         if inRasbery:
          if DIF_ANGLE < 0:
           dit.set_servo_pulsewidth(18,1500)       # устанавливаем в начальное положение
           #dit.set_servo_pulsewidth(27,1500)       # устанавливаем в начальное положение

         x0 = DIF_ANGLE + x0

         x1=x0
         y1=y0
         x = map_range(mob_range,robo_range_18,int(x1))       # переводим азимут в команду
         #y = map_range(mob_range,robo_range_27,int(y1))       # переводим тангаж в команду
         try:
          if inRasbery:
           if(dit.connected):
            dit.set_servo_pulsewidth(18,int(x))                  # отправляем команды на мотор
            #dit.set_servo_pulsewidth(27,int(y))
         except Exception as e:
          pass
    if mode == "mot":
     try:
      if inRasbery:

       if pwm1 == None:
         pwm1 = GPIO.PWM(22, 100)
         pwm1.start(0)
       if pwm2 == None:
         pwm2 = GPIO.PWM(25, 100)
         pwm2.start(0)
         print('motors on!')

       motComand = params[1]
       speedCommand = int(params[2])
       if motComand == "top":
            GPIO.output(27, True)
            GPIO.output(17, False)
            GPIO.output(24, True)
            GPIO.output(23, False)
            pwm1.ChangeDutyCycle(speedCommand)
            pwm2.ChangeDutyCycle(speedCommand)
       if motComand == "left":
            GPIO.output(27, True)
            GPIO.output(17, False)
            GPIO.output(24, False)
            GPIO.output(23, True)
            pwm1.ChangeDutyCycle(speedCommand)
            pwm2.ChangeDutyCycle(speedCommand)
       if motComand == "right":
            GPIO.output(27, False)
            GPIO.output(17, True)
            GPIO.output(24, True)
            GPIO.output(23, False)
            pwm1.ChangeDutyCycle(speedCommand)
            pwm2.ChangeDutyCycle(speedCommand)
       if motComand == "down":
            GPIO.output(27, False)
            GPIO.output(17, True)
            GPIO.output(24, False)
            GPIO.output(23, True)
            pwm1.ChangeDutyCycle(speedCommand)
            pwm2.ChangeDutyCycle(speedCommand)
       if motComand == "stop":
            GPIO.output(27, False)
            GPIO.output(17, False)
            GPIO.output(24, False)
            GPIO.output(23, False)
            pwm1.ChangeDutyCycle(0)
            pwm2.ChangeDutyCycle(0)
       
     except Exception as e:
      pass

@sio.event(namespace='/test')
def disconnect():
    print('disconnected from server')
    global dit
    global pwm1
    global pwm2
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
    if pwm1 != None:
         pwm1.stop()
         pwm1 = None
    if pwm2 != None:
         pwm2.stop()
         pwm2 = None
         print('motors stop!')

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

while True:
    try:
        sio.connect('https://evgenium.fvds.ru:5000') #адрес для коннекта
        sio.wait()
    except Exception as e:
        print(e)
