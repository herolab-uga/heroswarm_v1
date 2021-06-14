import socket

from motorControl import MotorControl as MC
import getch
MovCnt=MC()
HOST='172.22.87.11'
PORT=12346
CLIENT_SOCKET1=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
CLIENT_SOCKET1.connect((HOST,PORT))
print("1 Connected!!")
message1="I am the Client-1,1"
CLIENT_SOCKET1.send(message1.encode('ascii'))
while 1:
    ControlCommand=CLIENT_SOCKET1.recv(1024).decode('ascii')
    ctl=ControlCommand
    print(ctl)
    if(ctl=="w"):
        MovCnt.MoveForward()
        
    elif(ctl=="s"):
        MovCnt.MoveBackward()
        
    elif(ctl=="q"):
        MovCnt.Stopper()
    elif(ctl=="A"):
        MovCnt.TankSteerLeft()
    elif(ctl=="D"):
        MovCnt.TankSteerRight()
    elif(ctl=="a"):
        MovCnt.SteerLeft()
    elif(ctl=="d"):
        MovCnt.SteerRight()    
    elif(ctl=="R"):
        MovCnt.PWMClean()
    elif(ctl=="i"):
        MovCnt.IncreaseSpeed()
