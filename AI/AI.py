from math import cos, radians, sin
from time import time, time_ns
import requests
from scipy import interpolate
from collections import deque
import numpy as np
from AI.utils import*
from AI.UAV import UAV
import datetime
# from sympy import Q  # interesting library



#  # read from example telemetry response 
# telRes=0
# with open("exampleTelemetryResponse.json",'r') as file:
#     telRes=json.load(file)

#TODO:
#1: select optimal target = Done
#2: keep distance while chasing
#3: avoid crashes
#4: switch to kamikaze and back 

class DogFightAI:

    def __init__(self,params="defaultParams.npy",tuneEnable=False):
        self.friendlyTelemetry= deque(maxlen=DEQUE_MEMORY)
        self.fTelTimes=deque(maxlen=DEQUE_MEMORY)
        self.enemies= {}

        self.errorInfo=[[],[]]
        self.errorTime=[]
        self.prevErrs=[]
        self.prevTimes=[]
        self.learningRate=0.00004
        self.tuneTerms=[0,0]
        self.visLockCutoff=5

        self.timeOffset=time()
        self.PIDchanged=False
        self.tuneEnable=tuneEnable

        self.PID=np.load(params)

        
    def autoSelectTarget(self):
        scores=[]
        selfState=self.estimSelf(time())
        for enemy in list(self.enemies.keys()):
            dtime=datetime.now()
            dtMs=telTimetoMS(dtime)
            enemyState=self.enemies[enemy].estimateParams(dtMs)
            distance=distance3D(GPStoMeter(selfState[:3],enemyState[0]))
            scores.append([enemy,0.5*(distance-75)**2+2*abs(selfState[4]-enemyState[1][1])])
        try:
            scores.sort(key= lambda x: x[1])
            return scores[0]
        except:
            return None
    def updateEnemy(self,newEnemyTelemetry):
        # telemetry as json -> dictionary ->add to queue
        
        individualTels= newEnemyTelemetry["konumBilgileri"]
        if isinstance(individualTels,list):
            for tel in individualTels:
                newTel={"sunucusaati":newEnemyTelemetry["sunucusaati"],"konumBilgileri":tel}
                if not str(tel["takim_numarasi"]) in list(self.enemies.keys()):
                    self.enemies[str(tel["takim_numarasi"])]=UAV(tel["takim_numarasi"])
                self.enemies[str(tel["takim_numarasi"])].update(newTel)
        else:
            newTel={"sistemSaati":newEnemyTelemetry["sistemSaati"],"konumBilgileri":individualTels}
            if not str(individualTels["takim_numarasi"]) in self.enemies.keys():
                self.enemies[str(individualTels["takim_numarasi"])]=UAV(individualTels["takim_numarasi"])
            self.enemies[str(individualTels["takim_numarasi"])].update(newTel)

    
    def updateFriendly(self,newFriendlyTemeletry=None,lockError=None):
        if newFriendlyTemeletry!= None:
            '''
            beklenen format:
            [timeMS,[lat,lon,alt,pitch,yaw,roll]]
            '''
            self.friendlyTelemetry.append(newFriendlyTemeletry[1])
            self.fTelTimes.append(newFriendlyTemeletry[0])
        if lockError != None:
            self.errorInfo[0].append(lockError[0])
            self.errorInfo[1].append(lockError[1])
            self.errorTime.append(time())

    
    def estimSelf(self,t):
        if len(self.fTelTimes)>2:
            state=[
            interpolate.CubicSpline(self.fTelTimes,[x[0] for x in self.friendlyTelemetry])(t),
            interpolate.CubicSpline(self.fTelTimes,[x[1] for x in self.friendlyTelemetry])(t),
            interpolate.CubicSpline(self.fTelTimes,[x[2] for x in self.friendlyTelemetry])(t),
            interpolate.CubicSpline(self.fTelTimes,[x[3] for x in self.friendlyTelemetry])(t),
            interpolate.CubicSpline(self.fTelTimes,[x[4] for x in self.friendlyTelemetry])(t),
            interpolate.CubicSpline(self.fTelTimes,[x[5] for x in self.friendlyTelemetry])(t)]

            return state
        else :
            return [None for x in range(6)]
    def lockRelease(self):
        self.errorInfo.clear()
        self.errorTime.clear()
    def guessManuever(self,UAVid,t):
        # guesses what manuever the enemy plane is currently making and returns it along with the estimated next position of the enemy plane after t seconds
        pass
        
    def checkThreatLevels(self):
        # threat levels(collision): min-max=0.0-1.0
        distances=[]
        for enemy in self.enemies:
            distances.append(GPStoMeter(self.friendlyTelemetry[-1]["Location"]))


    def generateDodge(self,UAVid):
        # generate a target GPS coordinate for friendly plane to go to dodge the enemy plane and/or to avoid mid-air collision or reduce its likelihood
        pass
    
    def autoTune(self,terms):
        prevPeriod=self.prevTimes[-1]-self.prevTimes[0]
        prevPeriod=len(self.prevTimes)
        prevXmse=np.sum([x*x for x in self.prevErrs[0]])/prevPeriod
        prevYmse=np.sum([x*x for x in self.prevErrs[1]])/prevPeriod
        
        period=self.errorTime[-1]-self.errorTime[0]
        period=len(self.errorTime)
        Xmse=np.sum([x*x for x in self.errorInfo[0]])/period
        Ymse=np.sum([x*x for x in self.errorInfo[1]])/period

        derX=Xmse-prevXmse
        derY=Ymse-prevYmse
        print(derX,derY)
        self.PID[0][terms[0]]-=self.learningRate*derX
        self.PID[1][terms[1]]-=self.learningRate*derY
        try:
            savePath="PIDS/PID-"+str(time_ns())+".npy"
            np.save(savePath,np.array(self.PID))
        except:
            print(f"failed to save new PID:{self.PID}")



    def generateLockCommand(self,UAVid,visualLock=False):
        # generate PWM values and target GPS value to lock on target visually and physically
        xout=0
        yout=0
        if visualLock:
            if len(self.errorTime)>= 2:
                # print(f"err len:{len(self.errorTime)}, time len:{len(self.errorTime)}")
                dt=time()-self.errorTime[-1]
                # print(dt)
                if dt<self.visLockCutoff:
                    xcurve=interpolate.CubicSpline(self.errorTime,self.errorInfo[0])
                    ycurve=interpolate.CubicSpline(self.errorTime,self.errorInfo[1])
                    #Proportional
                    xout=self.PID[0][0]*xcurve(time())
                    yout=self.PID[1][0]*ycurve(time())
                    #Integral
                    xout+=self.PID[0][1]*xcurve.integrate(self.errorTime[0],time())
                    yout+=self.PID[1][1]*ycurve.integrate(self.errorTime[0],time())
                    #Derivative
                    xout+=self.PID[0][2]*xcurve.derivative()(time())
                    yout+=self.PID[1][2]*ycurve.derivative()(time())
                else:
                    if self.tuneEnable:
                        # print(np(self.prevErrs))
                        if len(self.prevTimes)>5 and len(self.errorTime)>5:
                            print("ehe")
                            if self.PIDchanged:
                                self.PIDchanged=False
                                self.PID[0][self.tuneTerms[0]]-=self.learningRate
                                self.PID[1][self.tuneTerms[1]]-=self.learningRate
                                self.autoTune(self.tuneTerms)
                                self.tuneTerms[0]+=1
                                self.tuneTerms[0]%=3
                                self.tuneTerms[1]+=1
                                self.tuneTerms[1]%=3
                            else:
                                self.PIDchanged=True
                                self.PID[0][self.tuneTerms[0]]+=(self.learningRate/10)
                                self.PID[1][self.tuneTerms[1]]+=(self.learningRate/10)
                        if len(self.errorTime) >0:
                            self.prevErrs=np.copy(self.errorInfo)
                            print(len(self.errorTime),np.shape(self.errorInfo))
                            self.prevTimes=np.copy(self.errorTime)
                    print("deleted")
                    self.errorInfo.clear()
                    self.errorInfo=[[],[]]
                    self.errorTime.clear()
            else:
                xout=0
                yout=0
        # print(xout,yout):
        try:
            dtime=requests.get("http://10.0.0.15:64559/api/sunucusaati")
            dtime=dtime.json()
        except:
            dtime=datetime.now()
            dtime=(dtime.hour-3)*3600*1000+dtime.minute*60*1000+dtime.second*1000+int(dtime.microsecond/1000)
        # print(dtime)
        timeMs=telTimetoMS(dtime)
        if  str(UAVid) in list(self.enemies.keys()):
            params= self.enemies[str(UAVid)].estimateParams(timeMs+1000)

            targetCoords=params[0]

            targetCoords[0]+= cos(params[1][1])*meterToGPS(50)
            targetCoords[1]+= sin(params[1][1])*meterToGPS(50)
            # print(f"yonelme: {params[1][1]}")
            targetCoords[2]+= params[0][2]
        else:
            print("rakip yok")
            print(list(self.enemies.keys()))
            targetCoords=[0,0,0]
        if xout!=0 and yout!=0:
            response=requests.get("http://192.168.31.90:4000/api/telemetrial")
            response=response.json()
            ihalat=response["IHA_enlem"]
            ihalon=response["IHA_boylam"]
            ihayaw=response["IHA_yonelme"]
            targetCoords=[ihalat,ihalon,70]
            targetCoords[0]+= cos(ihayaw)*meterToGPS(20)
            targetCoords[1]+= sin(ihayaw)*meterToGPS(20)
            targetCoords[2]=response["IHA_irtifa"]
        return [targetCoords,[xout,yout]]
        # return [[0,0,0],[xout,yout]]

        
