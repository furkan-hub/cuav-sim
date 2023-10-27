import requests
import functools
from time import time, sleep
from collections import deque

class COM:
    def __init__(self,serverConfig,rate=50) -> None:
        self.APIUrl="http://"+serverConfig["mini_api_ip"]+":"+serverConfig["mini_api_port"]
        self.RefURL=serverConfig["hakem_url"]

        self.CVState=[0,0,0,0,0]
        self.CVFlag=True

        self.QRState=[False,"no_data"]
        self.QRFlag=True

        self.refTime=None

        self.enemyTelemQ=deque(maxlen=20)

        self.mission=0
        self.target=None

        self.threadSafety=True
        self.period=1/rate

    def run(self):
        timer=time()
        while True:
            start=time()
            if self.threadSafety and self.CVFlag:
                CVCopy=self.CVState
                CVCopy=[int(x) for x in CVCopy]
                degSoz={"hedef_x":CVCopy[0],"hedef_y":CVCopy[1],"hedef_w":CVCopy[2],"hedef_h":CVCopy[3],"lock":CVCopy[4]}
                cvPost=requests.post(self.APIUrl+"/get_degsoz",json=degSoz)
                if cvPost.status_code==200:
                    self.CVFlag=False


            if self.threadSafety and self.QRFlag:
                QRCopy=self.QRState
                QRDict={"qr_check":int(QRCopy[0]),"qr_text":QRCopy[1]}
                qrPost=requests.post(self.APIUrl+"/get_degsoz_qr",json=QRDict)
                if qrPost.status_code==200:
                    self.QRFlag=False

            if self.threadSafety:
                timeGet=requests.get(self.APIUrl+"/sunucusaati_al")
                if timeGet.status_code==200 and self.checkError(timeGet.json(),msgType="time"):
                    self.refTime=timeGet.json()

            if self.threadSafety:
                telemGet=requests.get(self.APIUrl+"/api/rakip_telemetri_al")
                if telemGet.status_code==200 and self.checkError(telemGet.json(),msgType="telem"):
                    self.enemyTelemQ.append(telemGet.json())
            if self.threadSafety and timer<time():
                timer=time()+1
                missionStatus=requests.get(self.APIUrl+"/mod_mission")
                if missionStatus.status_code==200 :
                    self.mission=missionStatus.json()["mission"]
                
                target=requests.get(self.APIUrl+"/api/get_target")
                if target.status_code==200:
                    self.target=target.json()
            dt=time()-start
            if dt < self.period:
                sleep(self.period-dt)


    def setCVState(self,CVState):
        self.threadSafety=False
        if self.CVState != CVState:
            self.CVFlag=True
            self.CVState=CVState
        self.threadSafety=True

    def setQRState(self,QRState):
        self.threadSafety=False
        if self.QRState != QRState:
            self.QRFlag=True
            self.QRState=QRState
        self.threadSafety=True


    def getEnemyTelem(self):
        self.threadSafety=False
        temp= self.enemyTelemQ.copy()
        self.enemyTelemQ.clear()
        self.threadSafety=True
        return temp
        

    def getRefTime(self):
        return self.refTime
    
    def getMissionStatus(self):
        return {"mission":self.mission,"target":self.target}
    
    def checkError(self,msgJson,msgType):
        if msgType == "time":
            if msgJson!=self.refTime:
                return True
            return False
        elif msgType == "telem":
            if len(self.enemyTelemQ)==0:
                return True
            else:
                if self.enemyTelemQ[-1]["sunucusaati"]!=msgJson["sunucusaati"]:
                    return True
            return False