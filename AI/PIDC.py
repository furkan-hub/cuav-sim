import numpy as np
from scipy import interpolate
from scipy.signal import lfilter, butter
from collections import deque
from time import time
from math import copysign,pi
from AI.utils import clamp

class PID:
    def __init__(self,coeffs,offset=1500,limits=[1000,2000],forced=[0,0],filterKernel=0,slewRate=5000,iLim=None,dLim=None,sgnType=int,vis=0,filter_freq=2):
        '''coeffs=PID coefficients in order\n
        offset=signal offset\n
        limits=lower and upper limits on signal strength\n
        forced= first value is the threshold value. if the signal stregnth is less than or more than the threshold + offset, the second value is substracted/added to the signal\n
        slewRate= how much the output signal can change over 1 second \n
        sgnType= int or float
        '''
        n=120
        if vis:
            n=120
        
        self.errors=deque(maxlen=n)
        self.times=deque(maxlen=n)

        self.coeffs=coeffs
        if len(coeffs)<3:
            for _ in range(3-len(coeffs)):
                self.coeffs.append(0)
        self.errorCurve=None
        self.offset=offset
        self.limits=limits
        self.forced=forced
        self.filterKernel=filterKernel
        self.sgnType=sgnType
        self.slewRate=slewRate
        self.lastOutput=offset
        self.lastOTime=None
        self.iLim=None
        self.dLim=None
        self.outs=deque(maxlen=5)
        if vis:
            self.outs=deque(maxlen=n//6)
        if iLim!=-1:
            self.iLim=iLim
        if dLim!=-1:
            self.dLim=dLim
        self.filter_b, self.filter_a = butter(3,[0.5,5],btype='bandpass',fs=20)
    
    def update(self,error,t=None):
        self.errors.append(error)
        if t==None:
            self.times.append(time())
        else:
            self.times.append(t)
        if len(self.errors)>3:
            self.errorCurve=interpolate.CubicSpline(self.times,self.errors)

    def output(self,delay=0,coeff=None):
        if self.errorCurve==None:
            return self.offset
        out=0
        if self.coeffs[0] !=0:
            # out+=self.coeffs[0]*self.errorCurve(time()+delay)
            out+=self.coeffs[0]*self.errorCurve(self.times[-1])
        
        if self.coeffs[1] !=0:
            # integral=self.coeffs[1]*self.errorCurve.integrate(self.times[0],time()+delay)
            leakyErrors=[self.errors[i]/pow(len(self.errors)-i,0.5) for i in range(len(self.errors))]
            newCurve=interpolate.CubicSpline(self.times,leakyErrors)
            integral=newCurve.integrate(self.times[0],self.times[-1])
            if self.iLim!=None:
                integral=clamp(integral,-self.iLim,self.iLim)
            out+=self.coeffs[1]*integral
        if self.coeffs[2] !=0:
            der=self.errorCurve.derivative()
            ders=[]
            if self.filterKernel>0:
                filtered_derivative= lfilter(self.filter_b,self.filter_a,der(self.times))
                if len(filtered_derivative)>2:

                    ders.append(filtered_derivative[-len(filtered_derivative)//9:])
            # ders.append(der(time()+delay))
            else:

                ders.append(der(self.times[-1]))

            derOut=np.mean(ders)
            if self.dLim !=None:
                derOut=clamp(derOut,-self.dLim,self.dLim)
            out+=self.coeffs[2]*derOut
        
        if coeff!=None:
            out*=coeff
        out+=self.offset
        if out> self.offset+self.forced[0]:
            out+=self.forced[1]
        elif out< self.offset-self.forced[0]:
            out-=self.forced[1]


        if self.lastOTime==None:
            self.lastOTime=time()
            out=self.lastOutput
        else:
            dt=time()-self.lastOTime
            if dt>0.075:
                dt=0.075
            self.lastOTime=time()
            changeRate=out-self.lastOutput
            if abs(changeRate)>self.slewRate:
                out=self.lastOutput+self.slewRate*dt*copysign(1,changeRate)
            self.lastOutput=out
        out=self.sgnType(clamp(out,self.limits[0],self.limits[1]))
        self.outs.append(out)
        out=np.sum([self.errors[i]/pow((len(self.errors)-i),1.5) for i in range(len(self.errors))])/np.sum([1/pow((len(self.errors)-i),1.5) for i in range(len(self.errors))])
        return int(np.mean(self.outs))

    def clearMem(self):
        self.errorCurve=None
        self.errors.clear()
        self.times.clear()

    def updateParams(self,*args):
        args=[i for i in args]
        for i in range(len(args)):
            if args[i]==-1:
                args[i]=None
        self.coeffs=args[0]
        self.offset=args[1]
        self.limits=args[2]
        self.forced=args[3]
        self.filterKernel=args[4]
        self.slewRate=args[5]
        self.iLim=args[6]
        self.dLim=args[7]

    def errorState(self,delay=0):
        if len(self.errors)>0:
            return self.errors[-1]
        return 0