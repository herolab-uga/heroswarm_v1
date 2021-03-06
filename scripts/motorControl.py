import RPi.GPIO as gpio
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
        gpio.setup(3,gpio.OUT)
        gpio.setup(5,gpio.OUT)
        self.m1=gpio.PWM(3,45)
        self.m2=gpio.PWM(5,31)
        self.m1.start(0)
        self.m2.start(0)
    
    def MoveForward(self):
        self.m1.ChangeDutyCycle(12)
        self.m2.ChangeDutyCycle(3)
        
                
    
    def MoveBackward(self):
        self.m1.ChangeDutyCycle(3)
        self.m2.ChangeDutyCycle(12)
            
    def TankSteerLeft(self):
        self.m1.ChangeDutyCycle(3)
        self.m2.ChangeDutyCycle(3)
        
    
    def TankSteerRight(self):
        self.m1.ChangeDutyCycle(12)
        self.m2.ChangeDutyCycle(12)
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
