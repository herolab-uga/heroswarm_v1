from motorControl import MotorControl as MC
import getch
import time
MovCnt=MC()


for i in range(0,5):
    print(i)
    MovCnt.TankSteerLeft(rate=i)
    time.sleep(3)