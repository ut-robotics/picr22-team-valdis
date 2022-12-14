from nis import match
import numpy as np
import serial
import struct
import math

SEND_STRUCT = '<hhhHBH'
RCV_STRUCT = '<hhhhB'

class IRobotMotion:
    def open(self):
        global ser
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.050)
        if not ser.isOpen():
            ser.open()
    def close(self):
        
        ser.close()
    def move(self, speed):
        self.speed = speed
    def spin(self, direction, rot_speed):
        self.direction = direction
        self.rot_speed = rot_speed


class SerialMotion(IRobotMotion):
    def open(self):
        super(SerialMotion,self).open()
    def close(self):
        super(SerialMotion,self).close()
    def spin(self, x = 10):
        ser.write(struct.pack(SEND_STRUCT, -x, -x, -x, 0 ,1, 0xAAAA))
    
    def cords_to_send(self,x,y):
        '''
        000 000______________________________________848 000
           |                     |                      |
           |                     |                      |
           |                     |                      |
           |---------------------+----------------------|
           |                     |                      |
           |                     .                      |
           |                     |                      |
        000 480______________________________________848 480
    
        '''
        if (x - 424)**2 + (y - 1129)**2 - 654588 <= 0:
            spin_speed = 10
            if y > 385:
                spin_speed = 5
                ##ser.write(struct.pack(SEND_STRUCT, -30, 0, -30, 0 ,0, 0xAAAA))
                
            if x < 400:
                print("paremale")
                ser.write(struct.pack(SEND_STRUCT, spin_speed, spin_speed, spin_speed, 0 ,0, 0xAAAA))
            elif x > 460:
                print("Vasakule")
                ser.write(struct.pack(SEND_STRUCT, -spin_speed, -spin_speed, -spin_speed, 0 ,0, 0xAAAA))
        else:
            print("ringist  väljas")
            m_speed = 10   #kui lähedamal  
            if(y < 200):
                m_speed= 40
                
            if x < 300:
                ser.write(struct.pack(SEND_STRUCT, m_speed, 30, -m_speed, 0 ,0, 0xAAAA))
                print("vasak otse")
            elif x > 600:
                ser.write(struct.pack(SEND_STRUCT, m_speed, -30, -m_speed, 0 ,0, 0xAAAA))
                print("parem otse")
            else:
                ser.write(struct.pack(SEND_STRUCT, m_speed, 0, -m_speed, 0 ,0, 0xAAAA))
        '''if y > 100:
            print(y,"y")
            ser.write(struct.pack(SEND_STRUCT, 2, -2, 0, 0 ,0, 0xAAAA))'''
    def thro(self,dist,y):
        print(dist)
        ser.write(struct.pack(SEND_STRUCT, 40, 0, -40, 1000 ,0, 0xAAAA))
    def orbit(self, d = 20): # d == driection
        # nii siin tuleb trikitada
        # Sunday to do
        ser.write(struct.pack(SEND_STRUCT, int(-d/4), d , int(d/4), 0 ,1, 0xAAAA))
    def orbit_self_correct(self, d , x): # d == driection
        # nii siin tuleb trikitada
        # Markus do do
        
        # esimese mootori korrekeerimine kui x on ilus
        kor1 = int(d/6)
        
        # teise mootori korrekeerimine kui x on ilus
        kor3 = int(d/6)
        # kui x not in [400;460]
        if(x > 460):
            kor1 += 0
        elif(x < 400):
            kor3 -= 0
        
        ser.write(struct.pack(SEND_STRUCT, kor1, d , -kor3, 0 ,0, 0xAAAA))
    
    
    
    
class SerialOmniMotion():
    def __init__(self):
        self.wheel_angle = list(np.radians([120, 0, 240]))
        self.dist_center = 0.125
        self.wheel_speed_to_mpu = 59
        self.SEND_STRUCT = '<hhhHBH'
        self.open()
    
    def open(self):
        global ser
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.050)
        if not ser.isOpen():
            ser.open()
    def close(self):
        ser.close()
    
    def orbit(self,speed,y = 0):    
        #print("WE BOUT TO DO SPINN")
        self.send_to_robot(0.16 * speed,0,speed)
        
    def throw_ball(self,dist):
        dist = dist-100
        if(dist < 550):
            sped = 800
        else:
            #sped = 0.24 * dist + 625 - 50
            sped = 0.21 * dist + 625
        print(dist,"dist")
        print(sped,"sped")
        self.send_to_robot(0,0.75,0,int(sped))
        
    
    def move_robot(self, x_cord, y_cord):
        print()
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
        else:
            print("x ei muutu")
        
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
        else:
            print("y ei muutu")
                
        print(le_x,le_y,le_r ,"WHAT SEND")
        
        
        
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
        print(speeds)   
        ser.write(struct.pack(self.SEND_STRUCT, speeds[0],speeds[1],speeds[2], thro ,failsafe, 0xAAAA))
        
    def spin(self,i):
        self.send_to_robot(0,0,i)