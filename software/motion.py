from nis import match
import numpy as np
import serial
import struct
import math

SEND_STRUCT = '<hhhHBH'
RCV_STRUCT = '<hhhhB'
    
class SerialOmniMotion():
    def __init__(self):
        self.wheel_angle = list(np.radians([120, 0, 240]))
        self.dist_center = 0.125
        self.wheel_speed_to_mpu = 59 # EPIC CALCULATION
        self.SEND_STRUCT = '<hhhHBH'
        self.open()
    
    def open(self):
        global ser
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.050)
        if not ser.isOpen():
            ser.open()
    def close(self):
        ser.close()
    
    def orbit(self,speed, y = 0):    
        #print("WE BOUT TO DO SPINN")
        self.send_to_robot(0.19 * speed,0,speed)
        
    def throw_ball(self,dist):
        
        sped = 0.2 * dist + 700-25
        print(dist,"dist")
        print(sped,"sped")
        self.send_to_robot(0,0.5,0,int(sped))
        
    
    def move_robot(self, x_cord, y_cord):
        le_x = 0
        le_y = 0
        le_r = 0
        # x middle u 420 - 440 for now
        # y 400 - 360
        #print(x_cord,"x")
        if x_cord < 415:
            #print (x_cord - 415, "x_offset")
            # testimisel ->  offset / 500 = x to ask 
            le_x = (x_cord-415)/600
            le_x = min(-0.075,le_x)
            le_r = -0.5
            
        elif x_cord > 445:  
            #print( x_cord- 445, "x_offset")
            le_x = (x_cord-445)/600
            le_x = max(0.075,le_x)
            le_r = 0.5
    
        
        #print(y_cord,"y")  
        if y_cord > 440:
            #(" move back")
            le_y = (440 - y_cord)
            #print(le_y ,"y offset")
            le_y = le_y / 400
            le_y = min(-0.075,le_y)
        elif y_cord < 410:
            #print(" move forward")
            le_y = (410 - y_cord)
            #print (le_y,"y offset")
            le_y = le_y / 400
            le_y = max(0.075,le_y)
    
                
        # print(le_x,le_y,le_r ,"WHAT SEND")
        
        
        
        self.send_to_robot(le_x,le_y,0)
        
    # x küljele
    # y otse
    # ringi ringi
    def send_to_robot(self,speed_x,speed_y,speed_r,thro = 0,failsafe = 0 ):
        speeds = []
        # ROBOT SPEEDS
        robot_speed = math.sqrt(speed_x**2 + speed_y**2)
        robot_angle = math.atan2(speed_y, speed_x)
        # WHEEL SPEED
        for i in range(3):
            wheel_linear_vel = robot_speed * math.cos(robot_angle - self.wheel_angle[i] + self.dist_center * speed_r) + self.dist_center * speed_r
            wheel_angular_mpu = wheel_linear_vel * self.wheel_speed_to_mpu
            speeds.append(int(wheel_angular_mpu))
        
        ser.write(struct.pack(self.SEND_STRUCT, speeds[0],speeds[1],speeds[2], thro ,failsafe, 0xAAAA))
        
    def spin(self,i):
        self.send_to_robot(0,0,i)