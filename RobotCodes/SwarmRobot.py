# All linear distnaces in meters
import math
import time
import json
import atexit
import motorControl


class SwarmRobot:
    # A list of conversion factors
    unit_conversion = {
        "in":.0254,
        "cm":.01,
        "ft":.3048,
        "km":.001,
        "yd":1.8288,
        "mile":3218.69,
        "meters":1
    }

    def __init__(self,name,id,linear_speed,angular_speed,x=0.0,y=0.0,theta=0.0):
        # Names each robot
        self.name = name
        self.id = id
        # Current Robot Position
        self.current_pos = {
            "x":x,
            "y":y,
            "theta":theta
        }
        # Currenet Speed of bot this is set in the motor library for now
        self.current_speed = 0
        # Batttery Charge amount from 0 to 1
        self.battery_charge = 1
        # Linear Speed in meters per second
        self.linear_speed = linear_speed
        # Angular Speed in radians per second
        self.angular_speed = angular_speed
        self.mc = motorControl.MotorControl()
        # self.mc.Stopper()
        # atexit.register(self.save)

    def __del__(self):
        self.mc.Stopper()

    def save(self):
        with open("config/swarm_v1_config.JSON",'r+') as file:
            data = json.load(file)[self.name]
            data['x_pos'] = self.get_x()
            data['y_pos'] = self.get_y()
            data['theta'] = self.get_theta()
            json.dump(data, file, ensure_ascii=False, indent=4)

    def dist(self,pos_i,pos_f):
        delta_x = pos_f[0]- pos_i[0]
        delta_y = pos_f[1]- pos_i[1]
        return math.sqrt(delta_x**2 + delta_y**2)

    # Getters and Setters for the class instance variables
    def get_name(self):
        return self.name

    def set_name(self,name):
        self.name = name

    def get_id(self):
        return id

    def set_id(self,id):
        self.id = id

    def get_x(self):
        return self.current_pos["x"]

    def set_x(self,x):
        self.current_pos["x"] = x

    def get_y(self):
        return self.current_pos["y"]

    def set_y(self,y):
        self.current_pos["y"] = y

    def get_theta(self):
        return self.current_pos["theta"]

    def set_theta(self,theta):
        self.current_pos["theta"] = theta

    def get_linear_speed(self):
        return self.linear_speed

    def set_linear_speed(self,linear_speed):
        self.linear_speed = linear_speed

    def get_angular_speed(self):
        return self.linear_speed

    def set_angular_speed(self,angular_speed):
        self.linear_speed = angular_speed

    # Class String override
    def __str__(self):
        string = """
                Name: {name},\n
                ID" {robot_id}, \n
                Linear Speed: {linear_speed}, \n
                Angular Speed: {angular_speed}, \n
                X Position: {x}, \n
                Y Position: {y}, \n
                Theta: {theta} \n
                """ .format(
                    name = self.get_name(),
                    robot_id = self.get_id(),
                    linear_speed = self.get_linear_speed(),
                    angular_speed = self.get_angular_speed(),
                    x = self.get_x(),
                    y = self.get_y(),
                    theta = self.get_theta()
                )
        return string

    def reset_pos(self):
        self.current_pos["x"] = 0
        self.current_pos["y"] = 0
        self.current_pos["theta"] = 0

    def stop(self):
        self.mc.Stopper()

    # Moves the Robot Forward
    def forward(self,distance=None,unit="in"):
        if not distance == None:
            # Converts the distance to meters
            distance = distance * SwarmRobot.unit_conversion[unit]
            # Calculates the time needed to travel distance
            sleep = distance/self.linear_speed
            self.mc.MoveForward()
            # Waits for the amount of time needed to travel distance
            time.sleep(sleep)
            # Stops the robot when it is at the desired position
            self.mc.Stopper()
            # Calulates the current position of the robot based on the direction of travel
            self.current_pos["x"] = self.current_pos["x"] + math.cos(self.current_pos["theta"])*distance
            self.current_pos["y"] = self.current_pos["y"] + math.sin(self.current_pos["theta"])*distance
        else:
            self.mc.MoveForward()

    # Moves the Robot Backward
    def backward(self,distance=None,unit="in"):
        if not distance == None:
            # Converts the distance to meters
            distance = distance * SwarmRobot.unit_conversion[unit]
            # Calculates the time needed to travel distance
            sleep = distance/self.linear_speed
            self.mc.MoveBackward()
            # Waits for the amount of time needed to travel distance
            time.sleep(sleep)
                # Stops the robot when it is at the desired position
            self.mc.Stopper()
            # Calulates the current position of the robot based on the direction of travel
            self.current_pos["x"] = self.current_pos["x"] - math.cos(self.current_pos["theta"])*distance
            self.current_pos["y"] = self.current_pos["y"] - math.sin(self.current_pos["theta"])*distance
        else:
            self.mc.MoveBackward()

    def turn_left(self):
        self.mc.TankSteerLeft()

    def turn_right(self):
        self.mc.TankSteerRight()

    def turn(self,real=True,radians=True,angle=0):
        # Converts the angle to radians
        if not radians:
            angle = math.radians(angle)
        # Uses the real coordinate system to
        if real:
            # Finds the change in theta needed to be at angle
            delta_theta = angle - self.current_pos["theta"]
            # Finds the time needed to sleep to complete Angular
            sleep = abs(delta_theta)/self.angular_speed
            # If the angle is positive turn left
            if delta_theta > 0:
                self.mc.TankSteerLeft()
                time.sleep(sleep)
            # If the angle is negative turn right
            else:
                self.mc.TankSteerRight()
                time.sleep(sleep)
            # Updates the current postion
            self.current_pos["theta"] = angle
        # Uses a coordinate system relative to the robot where it is always facing theta=0
        else:
            # Finds the time need to sleep to turn through angle
            sleep = abs(angle)/self.angular_speed
            # If the angle is positive turn left
            if angle > 0:
                self.mc.TankSteerLeft()
                time.sleep(sleep)
            # If the angle is negative turn right
            else:
                self.mc.TankSteerRight()
                time.sleep(sleep)
            # Updates the current position
            self.current_pos["theta"] = self.current_pos["theta"] + angle
            # If the position is nagative after the update convert to positive unit circle angle
            if self.current_pos["theta"] < 0:
                self.current_pos["theta"] = 2*math.pi - self.current_pos["theta"]
        # Stopps the robot after turning
        self.mc.Stopper()

    # Moves the robot to position [x,y]
    def move_to(self,x,y,return_theta=None,unit="in"):
        # Converts the coordinates to meters
        x = x * SwarmRobot.unit_conversion[unit]
        y = y * SwarmRobot.unit_conversion[unit]
        # Gets the distance to move the robot
        move_distance = self.dist([self.current_pos["x"],self.current_pos["y"]],[x,y])
        # Gets the angle the robot needs to travel in to get to the position
        move_angle = math.atan2((y-self.current_pos["y"]),(x-self.current_pos["x"]))
        # Turns the robot to the correct heading
        self.turn(radians=True,angle=move_angle)
        # Moves the robot to the correct position
        self.forward(distance=move_distance,unit="meters")
        # Turns the robot back to the desired postion
        if not (return_theta == None):
            self.turn(angle=return_theta)

if __name__ == "__main__":
    test = SwarmRobot(name="Hiro",id=1,linear_speed=0.21082,angular_speed=7.0)
