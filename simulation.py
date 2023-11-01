from math import degrees
from ursina import *
from dronekit import *
from time import time
from ursina.shaders import lit_with_shadows_shader
import random

enAddress = "tcp:127.0.0.1:5772"
frAddress = "tcp:127.0.0.1:5762"

frUAV = connect(frAddress, timeout=30, wait_ready=False, rate=200)
print("fr")
enUAV = connect(enAddress, timeout=30, wait_ready=False, rate=200)
print("en1")

def GPStoMeter(loc1,loc2):
    # return the difference in location as meters
    return (loc2[0]-loc1[0])*111194.93, (loc2[1]-loc1[1])*111194.93 , loc2[2]-loc1[2]

def mod_change_fbwa(vehicle):
    print("test")
    vehicle.mode = "FBWA"

app = Ursina(title='Ursina',
    borderless=False,
    fullscreen=False)


lock_rectange = Button(color=color.clear, icon = "ui.png", scale=1)



crosshair = Entity(
    model='quad',
    color=color.white,
    scale=(0.02, 0.1),
    position=(0, 0)
)

player = Entity(
    model='uav',
    color=color.red,
    scale=2,
    shader=lit_with_shadows_shader
)

# Gökyüzü ekleyin.
sky = Entity(
    model='sphere',
    texture='sky.png',

    scale=3500,
    double_sided=True
)

# x left and right, pitch
player.x = 0
# y up and down, yaw
player.y = 0
# z forward backward, roll
player.z = 20
camera.fov = 40
timer = time()

def update():


    loc1 = [frUAV.location.global_relative_frame.lon, frUAV.location.global_relative_frame.lat, frUAV.location.global_relative_frame.alt]
    loc2 = [enUAV.location.global_relative_frame.lon, enUAV.location.global_relative_frame.lat, enUAV.location.global_relative_frame.alt]

    distVec = GPStoMeter(loc1, loc2)

    player.x = distVec[0]
    player.z = distVec[1]
    player.y = distVec[2]

    # angle in degrees
    camera.rotation = (-degrees(frUAV.attitude.pitch), degrees(frUAV.attitude.yaw), degrees(frUAV.attitude.roll))
    player.rotation = (-degrees(enUAV.attitude.pitch), -degrees(enUAV.attitude.yaw), degrees(enUAV.attitude.roll))

app.run()
