from math import degrees
from ursina import *
from dronekit import *
from time import time
from ursina.shaders import lit_with_shadows_shader
import requests
import json

Address = "tcp:127.0.0.1:5772"
server_url = "http://127.0.0.1:5000"

uav = connect(Address, timeout=30, wait_ready=False, rate=200)
print("fr")


def GPStoMeter(loc1, loc2):
    # return the difference in location as meters
    return (loc2[0]-loc1[0])*111194.93, (loc2[1]-loc1[1])*111194.93, loc2[2]-loc1[2]


def mod_change_fbwa(vehicle):
    print("test")
    vehicle.mode = "FBWA"


def create_telem(uav):

    lon = uav.location.global_relative_frame.lon
    lat = uav.location.global_relative_frame.lat
    alt = uav.location.global_relative_frame.alt

    pitch = uav.attitude.pitch
    yaw = uav.attitude.yaw
    roll = uav.attitude.roll

    telems = {

        "lon": lon,
        "lat": lat,
        "alt": alt,
        "pitch": pitch,
        "yaw": yaw,
        "roll": roll
    }

    return telems


app = Ursina(title='Ursina',
             borderless=False,
             fullscreen=False)

lock_rectange = Button(color=color.clear, icon="ui.png", scale=1)

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

    self_telem = create_telem(uav)
    json_telem_send = json.dumps(self_telem)

    requests.post(server_url+"/telem_post_2", json=json_telem_send)

    telempack = requests.get(
        server_url+"/telem_get_2")
    json_telem = telempack.json()
    print(json_telem)

    loc1 = [uav.location.global_relative_frame.lon,uav.location.global_relative_frame.lat, uav.location.global_relative_frame.alt]
    loc2 = [json_telem['lon'], json_telem['lat'], json_telem['alt']]

    distVec = GPStoMeter(loc1, loc2)

    player.x = distVec[0]
    player.z = distVec[1]
    player.y = distVec[2]

    # # angle in degrees
    camera.rotation = (-degrees(uav.attitude.pitch),degrees(uav.attitude.yaw), degrees(uav.attitude.roll))
    player.rotation = (-degrees(json_telem['pitch']),-degrees(json_telem['yaw']), degrees(json_telem['roll']))


app.run()
