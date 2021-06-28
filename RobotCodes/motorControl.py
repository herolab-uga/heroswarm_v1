import RPi.GPIO as gpio
import atexit
from time import sleep
class MotorControl:
    PWMValue=12
    m1=""
    m2=""
    pwmSpeed=50
    def __init__(self):
        #self.PWMValue=pwmTime
        gpio.setmode(gpio.BOARD)
        gpio.setup(40,gpio.IN)
        gpio.setup(33,gpio.OUT)
        gpio.setup(32,gpio.OUT)
        self.m1=gpio.PWM(33,35)
        self.m2=gpio.PWM(32,50)
        self.m1.start(0)
        self.m2.start(0)
        atexit.register(self.stop)

    def stop(self):
        gpio.cleanup()
    
    def MoveForward(self):
        self.m1.ChangeDutyCycle(12)
        self.m2.ChangeDutyCycle(3)
        
                
    
    def MoveBackward(self):
        self.m1.ChangeDutyCycle(3)
        self.m2.ChangeDutyCycle(12)
            
    def TankSteerLeft(self,rate=3):
        self.m1.ChangeDutyCycle(rate)
        self.m2.ChangeDutyCycle(rate)
        
    
    def TankSteerRight(self,rate):
        self.m1.ChangeDutyCycle(25-rate)
        self.m2.ChangeDutyCycle(25-rate)

    def IncreaseSpeed(self):
        self.pwmSpeed+=1
        self.m1=gpio.PWM(3,self.pwmSpeed)
        self.m2.gpio.PWM(5,self.pwmSpeed)
             

    def SteerLeft(self):
        self.m1.ChangeDutyCycle(0)
        self.m2.ChangeDutyCycle(3)
        
    
    def SteerRight(self):
        self.m1.ChangeDutyCycle(12)
        self.m2.ChangeDutyCycle(0)
        
    
    def Stopper(self):
        self.m1.ChangeDutyCycle(0)
        self.m2.ChangeDutyCycle(0)

    def PWMClean(self):
        self.m1.stop()
        self.m2.stop()
        gpio.cleanup()
