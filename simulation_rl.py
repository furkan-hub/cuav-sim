from math import degrees
from ursina import *           # this will import everything we need from ursina with just one line.
from dronekit import connect
import json
from AI.utils import *
from collections import deque
from time import time
from ursina.shaders import lit_with_shadows_shader 


enAddress="tcp:127.0.0.1:5770"

frAddress="tcp:127.0.0.1:5760"

# enAddress="tcp:192.168.137.241:5772"
# frAddress="tcp:192.168.137.241:5762"
frUAV=connect(frAddress,timeout=30,wait_ready=False,rate=100)
print("fr")
enUAV=connect(enAddress,timeout=30,wait_ready=False,rate=100)
print("en1")

def set_rc_channel(vehicle,rc_chan, value_us=0):  # kumanda kanallarini kontrol ediyor
    """
    anlamlari
    Input:
        rc_chan     - kumanda kanali numarasi
        value_us    - pwm degeri
    """
    strInChan = '%1d' % rc_chan
    vehicle.channels.overrides[strInChan] = int(value_us)

app = Ursina()

EditorCamera()



dists=[deque(maxlen=15),deque(maxlen=15),deque(maxlen=15)]
times=deque(maxlen=15)

Entity(model='plane', scale=10, color=color.gray, shader=lit_with_shadows_shader)



player = Entity(
    model = 'uav' ,           # finds a 3d model by name
    color = color.red,
    scale = 2,
    shader=lit_with_shadows_shader)

ehe=0
# x left and right, pitch
player.x=0
# y up and down, yaw
player.y=0
# z forward backward, roll
player.z=20
camera.fov=40
timer=time()
def update():                  # update gets automatically called by the engine.
    global ehe
    loc1=[frUAV.location.global_relative_frame.lon,frUAV.location.global_relative_frame.lat,frUAV.location.global_relative_frame.alt]
    loc2=[enUAV.location.global_relative_frame.lon,enUAV.location.global_relative_frame.lat,enUAV.location.global_relative_frame.alt]
   

    distVec=GPStoMeter(loc1,loc2)
    


    player.x=distVec[0]
    player.z=distVec[1]
    player.y=distVec[2]


    #angle in degrees
    camera.rotation=(-degrees(frUAV.attitude.pitch),degrees(frUAV.attitude.yaw),degrees(frUAV.attitude.roll))
   

    player.rotation=(-degrees(enUAV.attitude.pitch),degrees(enUAV.attitude.yaw),degrees(enUAV.attitude.roll))


    # camera.rotation=(10,0,0)


app.run()      