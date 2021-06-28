from motorControl import MotorControl as MC
import getch
import time
MovCnt=MC()


for i in range(40,70):
    print(i)
    MovCnt.TankSteerLeft(rate=i)
    input()