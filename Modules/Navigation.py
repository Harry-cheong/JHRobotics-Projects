#!/usr/bin/env python3

from ev3dev2.motor import OUTPUT_B, OUTPUT_C, MoveTank, MoveSteering, LargeMotor, SpeedPercent
from ev3dev2.led import Leds
from ev3dev2.button import Button
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.sensor import INPUT_2, INPUT_3
from time import *
import threading

#Sensors and Brick
leds = Leds()
buttons = Button()

run = threading.Event()

#Motors
tank = MoveTank(OUTPUT_C,OUTPUT_B)
tank.cs = ColorSensor(INPUT_3)

#start
leds.set_color('LEFT', 'RED')
leds.set_color('RIGHT', 'RED')
while not buttons.enter:
    pass


class Config():
    def __init__(self): 
        self.lm = LargeMotor(OUTPUT_B)
        self.rm = LargeMotor(OUTPUT_C)


        self.lls = ColorSensor(INPUT_2)
        self.rls = ColorSensor(INPUT_3)
        self.minRef = 0
        self.maxRef = 100

        self.steering = MoveSteering(OUTPUT_B, OUTPUT_C)
        self.movetank = MoveTank(OUTPUT_B, OUTPUT_C)
    
    def calibrate_ls(self,minRef,maxRef):
        self.minRef = minRef
        self.maxRef = maxRef 

class MotCtrl(Config):
    def __init__(self):
        
        #threading
        self.rlslock = threading.Lock()
        self.llslock = threading.Lock()
        self.event = threading.Event()
        
        #logging
        self.inters = False

        #inheritance 
        Config.__init__(self)

    def pid_follow(self,
                    kp,
                    ki,
                    kd,
                    speed,
                    pid_run = True,
                    target_light_intensity = None,
                    follow_left_edge = True,
                    right_light_sensor = True,
                    anticipate_intersection = False):
        
        if right_light_sensor:
            ls = self.rls
            self.rlslock.acquire()
        else:
            ls = self.lls
            self.llslock.acquire()
        
        ls.mode = 'COL-REFLECT'

        last_error = error = integral = 0.0
        derivative = 0.0
        
        while pid_run.is_set():

            error = target_light_intensity - (100 * ( ls.reflected_light_intensity - self.minRef ) / ( self.maxRef - self.minRef ))

            if error == 0:
                integral = 0
            else:
                integral = integral + error

            derivative = error - last_error
            last_error = error
            
            if self.inters:
                steering = 0
            else:
                steering = (kp * error) + (ki * integral) + (kd * derivative)

            self.steering.on(steering, SpeedPercent(speed))
            print(steering)
        
        self.rlslock.release() if right_light_sensor else self.llslock.release()
        

    def runpid(self,
                kp, 
                ki,
                kd,
                speed,
                Daemon = None,
                target_light_intensity = None, 
                follow_left_edge = True, 
                right_light_sensor = True,
                anticipate_intersection = True):

        self.event.set()
        self.pid = threading.Thread(target=self.pid_follow,args=(kp,ki,kd,speed), kwargs = {'target_light_intensity' : target_light_intensity, 'follow_left_edge' : follow_left_edge, 'right_light_sensor' : right_light_sensor, 'anticipate_intersection' : anticipate_intersection}, daemon = Daemon)
        self.pid.start()

        if detect_intersection:
            self.inters = threading.Thread(target=detect_intersection, args=(), daemon=None)

    def stop_pid(self):
        self.event.clear()
        self.pid.join()
        self.steering.off()

    def detect_intersection(self,args):
        try:
            self.llslock.acquire(blocking = False)
            ls = self.lls
        except:
            self.rlslock.acquire()
            ls = self.rls
        
        ls.mode = 'COL-COLOR'

        logging = []



        
        
        
        
        

    def MotorRegulation():
        pass

exe = MotCtrl()
exe.runpid(0.6,0,1,40,target_light_intensity = 20)
while not exe.lm.position > 3000:
    print(exe.lm.position)   
exe.stop_pid()
   
