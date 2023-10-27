from math import  radians,tanh,exp
from time import time
from scipy import interpolate
from collections import deque
import numpy as np
from AI.utils import*
from AI.UAV import UAV
from AI.PIDC import PID
from datetime import datetime
import json

class DogFightAI:

    def __init__(self,vehicle,params="parameters.json"):
        self.friendlyTelemetry= deque(maxlen=DEQUE_MEMORY)
        self.fTelTimes=deque(maxlen=DEQUE_MEMORY)
        self.enemies= {}
        self.enemyTypes={}
        self.vehicle=vehicle
        self.errorInfo=[[],[],[]]
        self.errorTime=[]
        self.refTime=None
        self.timeOffset=None

        self.visLockCutoff=1
        self.visResolution=[960,720]
        self.visFOV=[radians(24),radians(13.5)]
        self.visActive=False
        self.targetTelem={}
        self.telCounter=0
        self.routeTh=150
        self.dodge=False
        self.target=None
        self.circle2status=False
        self.pitchError=0
        self.yawError=0
        self.yaws=deque(maxlen=3)
        self.yaws.append(0)
        with open(params,'r') as jf:
            self.parameters=json.load(jf)

        self.PIDS={
            "speed":PID(*self.parameters["PIDs"]["speed"]),#
            "GPSSpeed":PID(*self.parameters["PIDs"]["GPSSpeed"]),#
            "visSpeed":PID(*self.parameters["PIDs"]["visSpeed"],int,1),

            "GPSYaw":PID(*self.parameters["PIDs"]["GPSYaw"]),#
            "GPSYawInv":PID(*self.parameters["PIDs"]["GPSYaw"]),#
            "GPSYawDodge":PID(*self.parameters["PIDs"]["GPSYaw"]),#
            "visYaw":PID(*self.parameters["PIDs"]["visYaw"],int,1),

            "GPSPitch":PID(*self.parameters["PIDs"]["GPSPitch"]),#
            "visPitch":PID(*self.parameters["PIDs"]["visPitch"],int,1),
            "altPitch":PID(*self.parameters["PIDs"]["altPitch"]),#

            "GPSRoll":PID(*self.parameters["PIDs"]["GPSRoll"])
        }
        self.alts=deque(maxlen=15)
        self.altTimes=deque(maxlen=15)

    def cTanh(self,x):
        return tanh(x*3)*pi/3
        
    def autoSelectTarget(self):
        """returns the optimal target for locking\n
        the target that is the closest to the locking distance and with the lowest yaw error is chosen"""
        scores=[]
        selfState=self.estimSelf(time())
        for enemy in list(self.enemies.keys()):
            dtime=datetime.now()
            dtMs=telTimetoMS(dtime)
            enemyState=self.enemies[enemy].estimateParams(dtMs)
            distance=distance3D(GPStoMeter(selfState[:3],enemyState[0]))
            dist=[  enemyState[0][0]-selfState[0],
                    enemyState[0][1]-selfState[1]]
        
            vecAngle=np.arctan2(*dist)
            delta=yawToEuclidian(np.mean(self.yaws))-vecAngle
            if delta>pi:
                delta-=2*pi
            elif delta <-pi:
                delta+=2*pi
            scores.append([enemy,abs(distance-self.parameters["plane"]["GPSDistance"])*abs(delta)])
        if len(scores)>0:
            scores.sort(key= lambda x: x[1])
            return scores[0][0]
        return None
    
    def updateDodge(self):
        error=0
        coeff=0
        params=[self.vehicle.location.global_relative_frame.lat,self.vehicle.location.global_relative_frame.lon,self.vehicle.location.global_relative_frame.alt]
        dtime=datetime.now()
        dtime=(dtime.hour)*3600*1000+dtime.minute*60*1000+dtime.second*1000+int(dtime.microsecond/1000)
        self.dodge=False
        for enemy in list(self.enemies.keys()) :
            if not isinstance(params[0],float):
                return
            distData=self.enemies[enemy].calcDist(params,dtime)
            if not isinstance(distData,list):
                continue
            
            
            if distData[0]<self.parameters["plane"]["dodgeDistance"] and distData[1]<self.parameters["plane"]["dodgeSpeed"]:
                self.dodge=True
                # print("ehe")
            # print(distData)
            temp=distData[2][:2]
            vecAngle=np.arctan2(*temp)
            distData[1]*=-1
            distData[0]=abs(distData[0])
            if distData[1]<1:
                distData[1]=1
            delta=(-yawToEuclidian(np.mean(self.yaws))+vecAngle)*distData[1]/distData[0]
            if delta>pi:
                delta-=2*pi
            elif delta <-pi:
                delta+=2*pi
            error+=delta
            coeff+=distData[1]/distData[0]
        if coeff>0:
            self.PIDS["GPSYawDodge"].update(error/coeff)

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
            newTel={"sunucusatti":newEnemyTelemetry["sunucusaati"],"konumBilgileri":individualTels}
            if not str(individualTels["takim_numarasi"]) in self.enemies.keys():
                self.enemies[str(individualTels["takim_numarasi"])]=UAV(individualTels["takim_numarasi"])
                self.enemyTypes[str(individualTels["takim_numarasi"])]=0
            self.enemies[str(individualTels["takim_numarasi"])].update(newTel)
        self.telCounter+=1
        if self.telCounter>self.routeTh:
            self.routeTh=30
            self.telCounter=0
            for key,value in self.enemies.items():
                if value.apprRoute():
                    ellparams=np.array(value.route.params[2:-1])*111194.93
                    if np.min(ellparams)<15:
                        self.enemyTypes[key]=2
                    else:
                        self.enemyTypes[key]=1
    def updateParams(self,newParams):
        self.parameters=newParams
        for key,value in self.PIDS.items():
            if "GPSYaw" in key:
                value.updateParams(*newParams["PIDs"]["GPSYaw"])
            else:
                value.updateParams(*newParams["PIDs"][key])
    

    
    def update(self):
        ### Autopilot
        #speed
        # speed=min(self.vehicle.groundspeed,self.vehicle.airspeed)
        speed=self.vehicle.groundspeed
        if not isinstance(speed,float):
            speed=22.0
        self.yaws.append(self.vehicle.attitude.yaw)
        speedError=0
        srange=self.parameters["plane"]["speedRange"]
        if speed < srange[0]:
            speedError=(srange[0]-speed)**2
        elif speed > srange[1]:
            speedError=-(speed-srange[1])**2
        else:
            speedError=self.parameters["plane"]["cruiseSpeed"]-speed

        self.PIDS["speed"].update(speedError)
        #pitch
        self.alts.append(self.vehicle.location.global_relative_frame.alt)
        self.altTimes.append(time())
        if len(self.alts)<3:
            self.PIDS["altPitch"].update(0)
        else:
            altCurve=interpolate.CubicSpline(self.altTimes,self.alts)
            altRange=self.parameters["plane"]["altitudeRange"]
            altRateRange=self.parameters["plane"]["altSpeedRange"]
            alt=altCurve(time())
            altRate=altCurve.derivative()(time())
            altPitchError=0
            if alt<altRange[0]:
                altPitchError+=altRange[0]-alt
            elif alt>altRange[1]:
                altPitchError+=altRange[1]-alt

            # if altRate<altRateRange[0]:
            #     altPitchError+=altRateRange[0]-altRate
            # elif altRate>altRateRange[1]:
            #     altPitchError-=altRateRange[1]-altRate
            self.PIDS["altPitch"].update(altPitchError)
    def updateGPS(self,target):
        self.target=target
        ### GPS yaw, pitch and speed errors
        ##### all requests must be async
        temp=self.timeOffset
        if self.timeOffset==None:
            temp=0
        timeMs=telTimetoMS(datetime.now())+temp
        if  str(target) in list(self.enemies.keys()):
            params= self.enemies[str(target)].estimateParams(timeMs+100)
            self.targetTelem={"lat":params[0][0],"lon":params[0][1],"alt":params[0][2],"roll":params[1][0],"pitch":params[1][1],"yaw":params[1][2]}
            targetCoords=params[0]
        else:
            print("rakip yok")
            print(list(self.enemies.keys()))
            return
        
        dist=[targetCoords[0]-self.vehicle.location.global_relative_frame.lat,
                targetCoords[1]-self.vehicle.location.global_relative_frame.lon]
        
        vecAngle=np.arctan2(*dist)
        delta=yawToEuclidian(np.mean(self.yaws))-vecAngle
        if delta>pi:
            delta-=2*pi
        elif delta <-pi:
            delta+=2*pi
        # lat1_rad = radians(self.vehicle.location.global_relative_frame.lat )
        # lon1_rad = radians(self.vehicle.location.global_relative_frame.lon )
        # lat2_rad = radians(targetCoords[0])
        # lon2_rad = radians(targetCoords[1])

        # # Next, calculate the difference in longitude:

        # delta_lon = lon2_rad - lon1_rad

        # # Now, we can compute the initial bearing (in radians) using the spherical law of cosines:

        # bearing_rad = np.arctan2(sin(delta_lon) * cos(lat2_rad), cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(delta_lon))

        # # Convert the bearing from radians to degrees:

        # bearing_deg = bearing_rad * (180 / pi)

        # # Finally, normalize the bearing to the range of -180 to 180 degrees:

        # normalized_yaw_angle = (bearing_deg + 180) % 360 - 180
        # normalized_yaw_angle = radians(normalized_yaw_angle)
        # delta=(normalized_yaw_angle-self.vehicle.attitude.yaw)
        # delta=self.cTanh(delta)
        # if abs(delta)<pi/15:
        #     delta*=1.25
        self.PIDS["GPSYaw"].update(clamp(delta,-pi/3,pi/3))
        self.PIDS["GPSYawInv"].update(-delta)
        altDif=targetCoords[2]-self.vehicle.location.global_relative_frame.alt
        distMetres=distance3D(GPStoMeter([0,0,0],[*dist,0]))
        #print(distMetres) 
        # pitchError=np.arctan2(altDif,distMetres)-self.vehicle.attitude.pitch
        # if abs(pitchError)<abs(altDif/10):
        #     pitchError=altDif/10
        self.PIDS["GPSPitch"].update(clamp(altDif,-20,20))
        absDist=abs(distMetres)-self.parameters["plane"]["GPSDistance"]
        distError=10/(1+exp(3-absDist/10))
        self.PIDS["GPSSpeed"].update(distError)
        self.targetTelem["yawError"]=delta
        self.targetTelem["altDif"]=altDif
        self.targetTelem["distance"]=distError
        # errorVecAngle=- np.arctan2(abs(pitchError),delta)+pi/2
        # self.PIDS["GPSRoll"].update(errorVecAngle)

    def updateCV(self,target,lockError):
        errorTime=lockError["time"]
        lockError=lockError["bbox"]
        # print(lockError)
        if lockError[2]==0 or lockError[3]==0:
            if time()>self.visLockCutoff and self.visLockCutoff!=-1:
                self.visActive=False
                self.PIDS["visSpeed"].clearMem()
                self.PIDS["visYaw"].clearMem()
                self.PIDS["visPitch"].clearMem()
                self.visLockCutoff=-1
            return
        else:
            if len(self.PIDS["visYaw"].errors)>10:
                self.visActive=True
                self.visLockCutoff=time()+2
        
        

        yawError=(lockError[0]/self.visResolution[0]-1/2)
        pitchError=(lockError[1]/self.visResolution[1]-1/2)

        # roll=self.vehicle.attitude.roll

        # nx= yawError*cos(roll)-pitchError*sin(roll)
        # ny=pitchError*cos(roll)+yawError*sin(roll)
        # yawError=nx
        # pitchError=ny
        yawError*=self.visFOV[0]/2
        pitchError*=self.visFOV[1]/2



        distError=min(lockError[2]/self.visResolution[0],lockError[3]/self.visResolution[1])
        ths=self.parameters["plane"]["visDistThs"]
        if distError < ths[1]:
            distError=ths[1]-distError
        elif distError > ths[0]:
            distError=distError-ths[0]
        else :
            distError=0
        # print(f"{yawError=} {pitchError=} {distError=}")
        self.PIDS["visSpeed"].update(distError,errorTime)
        self.PIDS["visYaw"].update(yawError,errorTime)
        self.PIDS["visPitch"].update(-pitchError,errorTime)
        


    
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
    


    def generateLockCommand(self):
        speedCoef=self.parameters["plane"]["cruiseSpeed"]/self.vehicle.groundspeed
        speedCoef=clamp(speedCoef,0.9,1.1)
        
        #TODO invert before flight
        autoPitch=self.PIDS["altPitch"].output(0.1)
        gpspitch=self.PIDS["GPSPitch"].output(0.1)#*(int(not self.visActive))
        vispitch=self.PIDS["visPitch"].output(0.1)*(int(self.visActive))
        # print(f"{autoPitch=} {gpspitch=} {vispitch=}")
        if self.visActive:
            pitch=clamp(1500+(autoPitch+gpspitch),1000,2000)
        else:
            pitch=clamp(1500+(autoPitch+gpspitch),1000,2000)
        
        # if self.dodge:
        #     GPSSpeed=0
        #     yaw=clamp(self.PIDS["GPSYawDodge"].output(0.1,speedCoef),1000,2000)
        # elif len(self.enemyTypes)>0 and self.enemyTypes[self.target]==2 and not self.circle2status:
        #     GPSSpeed=0
        #     yaw=clamp(self.PIDS["GPSYawInv"].output(0.1,speedCoef),1000,2000)
        #     dist=self.enemies[self.target].calcDist(self.vehicle.location.global_relative_frame,getTimeMS())
        #     if dist[0]>150:
        #         self.circle2status=True 
        #     elif dist[0]<self.parameters["plane"]["GPSDistance"]:
        #         self.circle2status=False

        if self.visActive:
            # print("viserrors",len(self.PIDS["visYaw"].errors))
            yaw=clamp(self.PIDS["visYaw"].output(),1000,2000)
            print(f"visyaw {yaw}")
            visSpeed=self.PIDS["visSpeed"].output(0.1)
        else:
            
            yaw=clamp(self.PIDS["GPSYaw"].output(0.1,speedCoef),1000,2000)
        GPSSpeed=self.PIDS["GPSSpeed"].output(0.1)
        if self.visActive:
            speed=clamp(self.PIDS["speed"].output(0.1)+GPSSpeed,1000,2000)
        else:
            speed=clamp(self.PIDS["speed"].output(0.1)+GPSSpeed,1400,2000)
        
        # print(self.visActive)
        
        return yaw,pitch,speed

        
    def guidedInputs(self,delay=0):
        speed=clamp(self.PIDS["speed"].output(0.1)+self.PIDS["GPSSpeed"].output(0.1),1000,2000)
        return [(self.PIDS["GPSYaw"].output(delay)-1500)*pi/500,(self.PIDS["GPSPitch"].output(delay))*pi/500,(speed-1000)/10]