from AI.utils import*
from collections import deque
from scipy import interpolate
from skimage.measure import EllipseModel
from math import radians

class UAV:
    def __init__(self,id):
        self.id=id


        self.pixelSize=deque(maxlen=DEQUE_MEMORY_VISION)
        self.physicalSize=deque(maxlen=DEQUE_MEMORY_VISION)

        self.location=[0,0,0]

        self.roll=0
        self.pitch=0
        self.yaw=0
        self.maxRoll=radians(30)
        self.distance=deque(maxlen=DEQUE_MEMORY)
        self.dtime=deque(maxlen=DEQUE_MEMORY)
        self.telemetry=deque(maxlen=DEQUE_MEMORY)
        self.timeDiffs=deque(maxlen=10)
        self.telTimes=[]
        self.locs=[]
        self.atts=[]
        self.route=EllipseModel()
    def update(self,newTelemetry):
        # print(self.telemetry)
        if len(self.telemetry)==0 or newTelemetry["sunucusaati"] != self.telemetry[-1]["sunucusaati"]:
            # print(newTelemetry)
            self.telemetry.append(newTelemetry)
            telTime=newTelemetry["sunucusaati"]
            self.timeDiffs.append(newTelemetry["konumBilgileri"]["zaman_farki"])
            self.telTimes.append(telTimetoMS(telTime,np.average(self.timeDiffs)))
            self.locs.append([newTelemetry["konumBilgileri"]["iha_enlem"],newTelemetry["konumBilgileri"]["iha_boylam"],newTelemetry["konumBilgileri"]["iha_irtifa"]])
            self.atts.append([newTelemetry["konumBilgileri"]["iha_yatis"],newTelemetry["konumBilgileri"]["iha_dikilme"],newTelemetry["konumBilgileri"]["iha_yonelme"]])
            curMaxRoll=max([abs(i[0]) for i in self.atts])
            if  curMaxRoll> self.maxRoll:
                self.maxRoll=(self.maxRoll+curMaxRoll)/2
            if len(self.telTimes)>2:
                x=10
                if len(self.telTimes)<10:
                    x=len(self.telTimes)
                self.location[0]= interpolate.CubicSpline(  self.telTimes[-x:],[i[0] for i  in self.locs[-x:]],bc_type='natural')
                self.location[1]= interpolate.CubicSpline(  self.telTimes[-x:],[i[1] for i  in self.locs[-x:]],bc_type='natural')
                self.location[2]= interpolate.CubicSpline(  self.telTimes[-x:],[i[2] for i  in self.locs[-x:]],bc_type='natural')
                self.roll   = interpolate.CubicSpline(      self.telTimes[-x:],[i[0] for i  in self.locs[-x:]],bc_type='natural')
                self.pitch  = interpolate.CubicSpline(      self.telTimes[-x:],[i[1] for i  in self.locs[-x:]],bc_type='natural')
                self.yaw    = interpolate.CubicSpline(      self.telTimes[-x:],[i[2] for i  in self.locs[-x:]],bc_type='natural')

    def calcDist(self,location,timeMS):
        dist=self.estimateParams(timeMS)[0]
        if dist[0]==0:
            return None
        if isinstance(location,list):
            distMeter=GPStoMeter(dist,location)
        else:
            distMeter=GPStoMeter(dist,[location.lat,location.lon,location.alt])
        self.distance.append(distance3D(distMeter))
        self.dtime.append(timeMS)
        if len(self.dtime)>2:
            distCurve=interpolate.CubicSpline(self.dtime,self.distance)
            return [distCurve(timeMS),distCurve.derivative()(timeMS),list(distMeter)]
        return [self.distance[-1],-25,list(distMeter)]

    def estimateParams(self,timeMs,fLoc=None):
        curLoc=[]
        curAngs=[]
        failure=False
        for loc in self.location:
            if not isinstance(loc,int):
                curLoc.append(loc(timeMs))
            else:
                
                failure=True
        if not isinstance(self.pitch,int):
            curAngs.append(self.pitch(timeMs))
        else:
            failure=True
        if not isinstance(self.yaw,int):
            curAngs.append(self.yaw(timeMs))
        else:
            failure=True
        if not isinstance(self.roll,int):
            curAngs.append(self.roll(timeMs))
        else:
            failure=True
        if failure:
            return [[0,0,0],[0,0,0]]
        else:
            #linear location interpolation based speed and yaw
            # curLoc[0]*=1
            # curLoc[1]*=1
            # latD=self.location[0].derivative()
            # lonD=self.location[1].derivative()
            # filteredSpeed=pow(pow(leakyMA(latD(self.telTimes)),2)+pow(leakyMA(lonD(self.telTimes)),2),0.5)
            # deltaT=timeMs-self.telTimes[-1]
            # linearLocation=[self.locs[-1][0]+cos(self.atts[-1][2])*meterToGPS(filteredSpeed*deltaT/1000),self.locs[-1][1]+sin(self.atts[-1][2])*meterToGPS(filteredSpeed*deltaT/1000)]
            # curLoc[0]+=linearLocation[0]*1
            # curLoc[1]+=linearLocation[1]*1

            # # location interpolation based on roll angle 
            
            # angularAcceleration=9.81*(abs(self.atts[-1][0])/self.maxRoll)
            # sn=np.sign(self.atts[-1][0])
            # centerYaw=self.atts[-1][2]+sn*pi/2
            # if centerYaw>pi:
            #     centerYaw-=2*pi
            # elif centerYaw <-pi:
            #     centerYaw+=2*pi

            # # # center=[self.locs[-1][0]+cos(centerYaw)*meterToGPS(radius),self.locs[-1][1]+sin(centerYaw)*meterToGPS(radius)]
            
            # radius=pow(filteredSpeed,2)/angularAcceleration
            # angularVel=filteredSpeed/radius
            # angularDelta=sn*angularVel*deltaT/1000
            # latDif=cos(angularDelta)*meterToGPS(radius)
            # lonDif=sin(angularDelta)*meterToGPS(radius)
            # rollLoc=[self.locs[-1][0]+latDif,self.locs[-1][1]+lonDif]

            # curLoc[0]+=rollLoc[0]
            # curLoc[1]+=rollLoc[1]
            # curLoc[0]/=2
            # curLoc[1]/=2
            
            return [curLoc,curAngs]

    def apprRoute(self):
        if len(self.telTimes)>150:
            xy=np.array([[i[0],i[1]] for i in self.locs])
            return self.route.estimate(xy)
        return False
           