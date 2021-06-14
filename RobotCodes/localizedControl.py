from motorControl import MotorControl as MC
import socket
import math
import numpy as np

MovCnt=MC()

def main():
    HOST = '192.168.1.78'  # Change this to server IP
    PORT = 65432        # Port to listen on (non-privileged ports are > 1023)
    robotSocket=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    robotSocket.connect((HOST, PORT))
    robotSocket.send(b'RobotConnected')              
    while True:
        robotSocket.send(b'ok')
        localData=robotSocket.recv(2048)
        
        #print(localData)
        setMotion1(localData)

def setMotion1(thetaData):
    thData=thetaData.decode('ascii')
    print(thData)
    if thData == 'Stop':
       MovCnt.Stopper()
    else:
       MovOnTheta(int(float(thData)))

        
def setMotion(data):
    thetaMargin=10
    #print(data)
    dta=data.decode('ascii').split()
    stpFlag=False
    if not len(dta)==0:
        x=int(float(dta[0]))
        y=int(float(dta[1]))
        hx=int(float(dta[2]))
        hy=int(float(dta[3]))
        eStatus= dta[4]=='True'
        #print(eStatus)
        ex=int(float(dta[5]))
        ey=int(float(dta[6]))
        trgDist=(math.sqrt((x-ex)**2)+(y-ey)**2)
        #print(trgDist)
        strtPt=np.array([x,y])
        endPt=np.array([ex,ey])
        headDir=np.array([hx,hy])
        theta =getTheta(strtPt,endPt,strtPt,headDir)
        theta=int(theta)
        #print(theta)
        if(trgDist<-1000):
            stpFlag=True
        else:
            stpFlag=False
        MovOnTheta(theta)

def MovOnTheta (theta):
     stpFlag=False

     eStatus=True
     thetaMargin=20        
     if not stpFlag and eStatus:     
          if theta>thetaMargin and theta<180:
            #print('Turn Left')
            MovCnt.SteerLeft()
          elif theta<360-thetaMargin and theta >=180 :
            #print('Turn Right')
            MovCnt.SteerRight()
          else:
            #print('Go Straight')
            MovCnt.MoveForward()
     else:
          #print('Stop')
          MovCnt.Stopper()





def getTheta(pt11,pt12,pt21,pt22):
    vec1=pt11-pt12
    vec2=pt22-pt21

    #vec12ds=math.degrees(math.asin2(np.cross(vec1,vec2)/(np.linalg.norm(vec1)*np.linalg.norm(vec2))))
    #vec12dc=math.degrees(math.asin2(np.cross(vec1,vec2)/(np.linalg.norm(vec1)*np.linalg.norm(vec2))))

    vec12dt=(math.degrees(math.atan2(np.cross(vec1,vec2),np.dot(vec1,vec2))))+180


    return vec12dt

if __name__ == '__main__':
    main()


