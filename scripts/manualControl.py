from motorControl import MotorControl as MC
import getch
MovCnt=MC()

while 1:
    ctl=getch.getch()
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
