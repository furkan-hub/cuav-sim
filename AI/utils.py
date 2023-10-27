from datetime import datetime
import numpy as np
from math import pi, sin,cos,acos
import requests
DEQUE_MEMORY=10
DEQUE_MEMORY_VISION=200
FOV=[60,45]
CAMERA_RESOLUTION=[640,480]
G=9.81
def GPStoMeter(loc1,loc2):
    # return the difference in location as meters
    return (loc2[0]-loc1[0])*111194.93, (loc2[1]-loc1[1])*111194.93 , loc2[2]-loc1[2]

def telTimetoMS(telTime,delta=0):
    if isinstance(telTime,dict):
        return telTime["saat"]*3600*1000+telTime["dakika"]*60*1000+telTime["saniye"]*1000+telTime["milisaniye"]-delta
    elif isinstance(telTime,datetime):
        return telTime.hour*3600*1000+telTime.minute*60*1000+telTime.second*1000+telTime.microsecond/1000

def calcLAH(array):
    #return the lowest, average and highest values of an array
    nArray=np.copy(array)
    nArray.sort()
    np.average(nArray)
    return [nArray[0],np.average(nArray),nArray[-1]]
def distance3D(vec):
    return (vec[0]**2 + vec[1]**2 + vec[2]**2)**.5

def meterToGPS(distance):
    return distance/111194.93

def spherToCartes(r,theta,phi):

    x=r*sin(phi)*cos(theta)
    y=r*sin(phi)*sin(theta)
    z=r*cos(phi)

    return [x,y,z]

def cartesToSpher(x,y,z):
    r=distance3D([x,y,z])
    theta=acos(z/r)
    # print(z,r,theta)
    phi=np.arctan2(y,x)

    return [r,theta,phi]

def yawToEuclidian(angle):
    angle*=-1
    angle+=pi/2
    return angle

def eucToYaw(angle):
    angle-=pi/2
    angle*=-1
    return angle

def cart2sph(x, y, z):
    hxy = np.hypot(x, y)
    r = np.hypot(hxy, z)
    el = np.arctan2(z, hxy)
    az = np.arctan2(y, x)
    return [r ,az, el]

def clamp(val,low,high):
    if val<low:
        val=low
    elif val>high:
        val=high
    return val

def getTimeMS():
        try:
            dtime=requests.get("http://192.168.43.41:8000/api/sunucusaati")
            dtime=dtime.json()
        except:
            dtime=datetime.now()
            dtime=(dtime.hour)*3600*1000+dtime.minute*60*1000+dtime.second*1000+int(dtime.microsecond/1000)
        # print(dtime)
        if type(dtime)!=int:
            timeMs=telTimetoMS(dtime)
        else:
            timeMs=dtime
        return timeMs

def leakyMA(array):
    vector=np.linspace(0,1,len(array))
    return np.sum(np.array(array)*vector)/np.sum(vector)