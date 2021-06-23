from __future__ import division
from __future__ import print_function
from argparse import ArgumentParser
import enum
import threading
import getch
import socket, pickle
import cv2
import apriltag
import math
import numpy as np
import time



# Handling through dictionaries
odoData={'robot1': 170, 'robot2': 650}
endPtID='4'


def QueryHandler(query,clientID):
    global odoData
    if query=='a':
        requestedData=odoData
    elif query=='o':
        requestedData=odoData.get(clientID)
    elif query=='e':
        requestedData=odoData.get(endPtID)
    else:
        requestedData='nah'



    return requestedData


def ThreadedConnection(connectedClient):
    clientID=connectedClient.recv(1024).decode('ascii')
    while True:
        dataToSend=QueryHandler(connectedClient.recv(1024).decode('ascii'),clientID)
        dataToSendP=pickle.dumps(dataToSend)
        connectedClient.send(dataToSendP)

def Main():
    orig_id = 2
    ref_y_id = 11
    ref_x_id = 6
    x_distance = 2.0447
    y_distance = 1.2192
    maxBots=1
    global odoData
    print(odoData)
    odoData1={'robot1': 170, 'robot2': 650}
    window = 'Overlay1'

    cv2.namedWindow(window)



    # Localization Code Here
    parser = ArgumentParser(description='test apriltag Python bindings')
    parser.add_argument('device_or_movie', metavar='INPUT', nargs='?', default=0, help='Movie to load or integer ID of camera device')
    orient = 0
    apriltag.add_arguments(parser)
    options = parser.parse_args()
    try:
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    except ValueError:
        cap = cv2.VideoCapture(options.device_or_movie)


    detector = apriltag.Detector(options,
                                searchpath=apriltag._get_demo_searchpath())
    font                   = cv2.FONT_HERSHEY_SIMPLEX
    fontScale              = 0.3
    fontColor              = (0,0,255)
    lineType               = 1
    while True:
        odoData1.clear()
        success, frame = cap.read()
        if not success:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        detections, dimg = detector.detect(gray, return_image=True)
        headDir=np.array([0,0])
        num_detections = len(detections)
        dimg1=dimg

        ref_x = None
        ref_y = None
        orig = None
        transform = []

        for k,tag in enumerate(detections):
            # print(type(tag.tag_id))
            if tag.tag_id == ref_x_id:
                ref_x = tag.center
                # print("x")
            elif tag.tag_id == ref_y_id:
                ref_y = tag.center
                # print("y")
            elif tag.tag_id == orig_id:
                orig = tag.center
                # print("orig")
        
        if ref_y is None or ref_x is None or orig is None:
            print("Showing")
            continue
        
        transform.append(np.abs(x_distance)/np.abs(ref_x[0]-orig[0]))
        transform.append(np.abs(y_distance)/np.abs(orig[1] -ref_y[1]))

        print(transform)

        for i, detection in enumerate(detections):
            dimg1 = draw(frame,detection.corners)
            center=detection.center
            center_meters = (transform[0] * (float(center[0]) - orig[0]),transform[1] * (orig[1] - float(center[1])) )
            print(center)
            posString = "({x:.2f},{y:.2f})".format(x=center_meters[0],y=center_meters[1])
            forwardDir,angle=headingDir(detection.corners,center)
            centerTxt=((center.ravel()).astype(int)).astype(str)
            dimg1=draw1(dimg1,forwardDir,center,(0,0,255))
            cv2.putText(dimg1,posString, tuple((center.ravel()).astype(int)+10),font,fontScale,(255,0,0),lineType)



            if detection.tag_id==endPtID:
                dimg1=cv2.putText(dimg1,'End Point', tuple((center.ravel()).astype(int)),font,0.8,fontColor,2)
                dimg1 = cv2.circle(dimg1, tuple((center.ravel()).astype(int)),30, (0,128,255), 2)
            else:
                cv2.putText(dimg1,'Id:'+str(detection.tag_id), tuple((center.ravel()).astype(int)),font,0.8,(0,0,0),2)



            overlay=dimg1
            odoData1[str(detection.tag_id)]=[tuple(center_meters)+tuple(forwardDir),angle]
        if(len(detections)==0 and odoData1==0):
            overlay=frame
            odoData=odoData1.copy()

        elif odoData1:
            odoData=odoData1.copy()


        #print('frame')

        cv2.imshow(window, overlay)
        cv2.waitKey(1)
# Change the following line to get back the connection part.



    #sndString=str(strtPt[0])+' '+str(strtPt[1])+' '+str(headDir[0])+' '+str(headDir[1])+' '+str(endptFlag)+' '+str(endpt[0])+' '+str(endpt[1])

    #k = cv2.waitKey(1)











def getTheta(pt11,pt12,pt21,pt22):
    vec1=pt11-pt12
    vec2=pt22-pt21
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
    midPt=(corner1+corner2)/2
    distance = math.sqrt( (abs(midPt[0])**2)+(abs(midPt[1])**2) )
    cMidPt=center-midPt
    thta=math.degrees(math.atan2(cMidPt[1],cMidPt[0]))
    cMidPt[0]=cMidPt[0]+50*math.cos(math.radians(thta))
    cMidPt[1]=cMidPt[1]+50*math.sin(math.radians(thta))
    newmidPt=cMidPt+center
    return newmidPt,thta





if __name__ == '__main__':
    Main()
