import socketio
import datetime
import time
from threading import Thread
import rclpy

from CollisionAvoidanceManager import CollisionAvoidanceManager, Cfg as CoavCfg
from ChassisInterface import ChassisInterface, Protocol

pipeline = None
bus = None
messageGs = None

ads = None
PAUSE_START_VIDEO = 0.0
pro=None
inRasbery = True
sio = socketio.Client()
start_time = time.time()
connected = False
ms = 0
video = None
g_speed_command = 0
g_mot_command = "stop"
g_coav = None

#перевод углов в команды, а - углы, b - сигналы, s - угол (вызов из test_broadcast_message)
def map_range(a,b,s): 
    (a1,a2),(b1,b2)=a,b
    return b1 + ((s-a1)*(b2-b1))/(a2-a1)


@sio.event(namespace='/test') #действия при запуске
def connect():
    global ads
    global connected
    global ms
    global pipeline
    global g_coav
    ping_pongStart()    
    connected = True
    print('connection established')
    try:
        while connected:   #пингуем раз в секунду
            sio.sleep(1)
            ping_pongR(ms)
            data = g_coav.GetRange(CoavCfg.RangeId.Ultrasonic1), g_coav.GetRange(CoavCfg.RangeId.Ultrasonic2), g_coav.GetRange(CoavCfg.RangeId.Ultrasonic3), g_coav.GetRange(CoavCfg.RangeId.Ultrasonic4)
            ping_pongU(data)
#            print('UltS = ', data)
            print('Ult1 = ', g_coav.GetRange(CoavCfg.RangeId.Ultrasonic1))
            print('Ult2 = ', g_coav.GetRange(CoavCfg.RangeId.Ultrasonic2))
            print('Ult3 = ', g_coav.GetRange(CoavCfg.RangeId.Ultrasonic3))
            print('Ult4 = ', g_coav.GetRange(CoavCfg.RangeId.Ultrasonic4))
            print('###########################@@@@@@@@@@@@@')
    except KeyboardInterrupt:
        print('interrupted!')

def handle_motion_command_i2c(motComand, speedCommand):
    global g_coav

    print("handle_motion_command_i2c {0} {1}\r\n".format(motComand, speedCommand))
    g_coav.updateCmd(motComand, speedCommand)

#ОБРАБОТКА управляющих сообщений от сервера (запускается после получения сообщения от сервера)
@sio.on('my_responseIO', namespace='/test')
def test_broadcast_message(data):
    print('message received with ', data)
    global video
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
    global connected
    connected = False
    handle_motion_command_i2c("stop", 0)
    

def ping_pongStart():
    global start_time
    start_time = datetime.datetime.utcnow()
    sio.emit('my_pingR', {u'data': u''}, namespace='/test')
    
#ответ на событие my_pongR с сервера - расчет задержки
@sio.on('my_pongR', namespace='/test')
def latencyR(data):
    global start_time
    global ping_pong_time
    global ms
    ping_pong_time = (str)((datetime.datetime.utcnow() - start_time)/2)
    secondS = ping_pong_time.split(':')
    ms = round(float(secondS[2])*1000, 2)
#    print('задержка = ', ms)
    return ms

#передача данных на сервер
def ping_pongR(data):
    global start_time
    start_time = datetime.datetime.utcnow()
    sio.emit('my_pingR', {u'data': data}, namespace='/test')

#передача данных на сервер вольтаж 
def ping_pongV(data):
    sio.emit('my_pingV', {u'data': data}, namespace='/test')

#передача данных на сервер временно УЗ
def ping_pongU(data):
    sio.emit('my_pingU1', {u'data': data}, namespace='/test')


def main_remote():
    global g_coav

    try:
        rclpy.init()
        g_coav = CollisionAvoidanceManager()

        thread = Thread(target=rclpy.spin, args=(g_coav._rosSub,), daemon=True)
        thread.start()

        while True:
            try:
                sio.connect('https://evgenium.fvds.ru:5000') #адрес для коннекта
                sio.wait()
            except ConnectionRefusedError as ex: 
                print(ex)

    except KeyboardInterrupt as ex:
        print("Shutdown on Ctrl+C")
    except Exception as ex:
        print("Exception : {}".format(ex))
    finally:
        rclpy.shutdown()        

main_remote()
