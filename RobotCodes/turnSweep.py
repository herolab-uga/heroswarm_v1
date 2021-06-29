from motorControl import MotorControl as MC
import getch
import time
MovCnt=MC()


for i in range(40,55):
    print(i)
    MovCnt.TankSteerLeft(49)
    time.sleep(3)
    MovCnt.TankSteerLeft(60)
    time.sleep(3)

    # input()