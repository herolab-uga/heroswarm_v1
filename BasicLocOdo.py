
'''Demonstrate Python wrapper of C apriltag library by running on camera frames.'''
from __future__ import division
from __future__ import print_function

from argparse import ArgumentParser
import cv2

import apriltag
import math
import numpy as np
import socket


def main():
    transmitCntr=0
    hostname = socket.gethostname()
    ip_address = socket.gethostbyname(hostname)
    HOST = '192.168.1.78'  # The server's hostname or IP address # This is the server
    PORT = 65432       # The port used by the server
    print(ip_address)
    serverSocket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    serverSocket.bind((HOST,PORT))
    serverSocket.listen()
    
    robotClient,addr=serverSocket.accept()
    msg=robotClient.recv(1024).decode('ascii')

    if(msg=='RobotConnected'):
        print('Starting Localization for 1 robot')

        parser = ArgumentParser(
            description='test apriltag Python bindings')

        parser.add_argument('device_or_movie', metavar='INPUT', nargs='?', default=0,
                            help='Movie to load or integer ID of camera device')
        orient = 0
        apriltag.add_arguments(parser)

        options = parser.parse_args()

        try:
            cap = cv2.VideoCapture(0)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)
        except ValueError:
            cap = cv2.VideoCapture(options.device_or_movie)

        window = 'Camera'
        window2 = 'Overlay1'
        cv2.namedWindow(window)
        cv2.namedWindow(window2)

        detector = apriltag.Detector(options,
                                    searchpath=apriltag._get_demo_searchpath())

        font                   = cv2.FONT_HERSHEY_SIMPLEX
        fontScale              = 0.3
        fontColor              = (0,0,255)
        lineType               = 1
        print(type(fontColor))
        while True:
            
            
            endpt=[0,0]
            strtPt=[0,0]
            strtFlag=False
            endptFlag=False
            success, frame = cap.read()
            if not success:
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            detections, dimg = detector.detect(gray, return_image=True)
            headDir=np.array([0,0])
            num_detections = len(detections)
            #print('Detected {} tags.\n'.format(num_detections))
            #dimg=np.zeros
        
            dimg1=dimg

            for i, detection in enumerate(detections):
                dimg1 = draw(frame,detection.corners)
                center=detection.center
                centerTxt=((center.ravel()).astype(int)).astype(str)
                if detection.tag_id==6:
                    strtPt=center
                    strtFlag=True
                    headDir=headingDir(detection.corners,center)
                if detection.tag_id==9:
                    dimg1=cv2.putText(dimg1,'End Point', tuple((center.ravel()).astype(int)),font,0.8,fontColor,2)
                    dimg1 = cv2.circle(dimg1, tuple((center.ravel()).astype(int)),30, (0,128,255), 2) 
                    endpt=center
                    endptFlag=True
                else:
                    cv2.putText(dimg1,'Id:'+str(detection.tag_id), tuple((center.ravel()).astype(int)),font,0.8,(0,0,0),2)
                    botDir=headingDir(detection.corners,center)
                    dimg1=draw1(dimg1,botDir,center,(0,0,255))
                cv2.putText(dimg1,'('+centerTxt[0]+','+centerTxt[1]+')', tuple((center.ravel()).astype(int)+10),font,fontScale,(255,0,0),lineType)
            if num_detections >0:
                if endptFlag and strtFlag:
                    orient=getTheta(strtPt,endpt,strtPt,headDir)
                    dimg1=cv2.putText(dimg1,'T:'+str(int(orient)), tuple((strtPt.ravel()).astype(int)+50),font,0.8,(0,0,0),2)
                    dimg1=draw1ine(dimg1,strtPt,endpt,(255,0,255))
                overlay=dimg1
            else: 
                overlay = frame

        # Change the following line to get back the connection part.
             
            transmitCntr=0
            sndString=str(strtPt[0])+' '+str(strtPt[1])+' '+str(headDir[0])+' '+str(headDir[1])+' '+str(endptFlag)+' '+str(endpt[0])+' '+str(endpt[1])
            
            if robotClient.recv(2048).decode('ascii')=='ok':
                robotClient.send(CalTheta(sndString).encode('ascii'))

        


            cv2.imshow(window, overlay)
            #k = cv2.waitKey(1)



def CalTheta(posString):
    stpFlag=False
    dta=posString.split()
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
        theta1 =getTheta(strtPt,endPt,strtPt,headDir)
        #print(theta)
        if(trgDist<-2000 or trgDist >=100000):
            theta="Stop"
        else:
            theta=str(theta1)
    print(trgDist)
    print(theta)
    return theta



def getTheta(pt11,pt12,pt21,pt22):
    vec1=pt11-pt12
    vec2=pt22-pt21

    #vec12ds=math.degrees(math.asin2(np.cross(vec1,vec2)/(np.linalg.norm(vec1)*np.linalg.norm(vec2))))
    #vec12dc=math.degrees(math.asin2(np.cross(vec1,vec2)/(np.linalg.norm(vec1)*np.linalg.norm(vec2))))

    vec12dt=(math.degrees(math.atan2(np.cross(vec1,vec2),np.dot(vec1,vec2))))+180


    return vec12dt




def draw1ine(img,point1,point2,clr):
    corner1 = tuple((point1.ravel()).astype(int))
    corner2=tuple((point2.ravel()).astype(int))
    img = cv2.line(img, corner2, corner1, clr, 1)
    return img


def draw1(img,point1,point2,clr):
    corner1 = tuple((point1.ravel()).astype(int))
    corner2=tuple((point2.ravel()).astype(int))
    img = cv2.arrowedLine(img, corner2, corner1, clr, 2)
    return img

def draw(img, corners):
    corner1 = tuple((corners[0].ravel()).astype(int))
    corner2=tuple((corners[1].ravel()).astype(int))
    corner3=tuple((corners[3].ravel()).astype(int))


    img = cv2.line(img, corner1, corner2, (255,0,0), 2)
    img = cv2.line(img, corner1, corner3, (0,255,0), 2)

    return img

def headingDir(corners,center):
    


    corner1 = (corners[0].ravel())
    
    corner2=(corners[1].ravel())

    #print(corner1)
    #print(corner2)
    
    midPt=(corner1+corner2)/2
    #print(midPt)
    distance = math.sqrt( (abs(midPt[0])**2)+(abs(midPt[1])**2) )
    cMidPt=center-midPt
    thta=math.degrees(math.atan2(cMidPt[1],cMidPt[0]))
    cMidPt[0]=cMidPt[0]+50*math.cos(math.radians(thta))
    cMidPt[1]=cMidPt[1]+50*math.sin(math.radians(thta))

    newmidPt=cMidPt+center
    
    #print(newmidPt)

    return newmidPt


if __name__ == '__main__':
    main()
