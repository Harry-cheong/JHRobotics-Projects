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

class LightSensor(ColorSensor):
    def __init__(self):
        self.minRef = 0
        self.maxRef = 100

        ColorSensor.__init__(self)

    def calibrate_ref(self, minRef, maxRef):
        self.minRef = minRef
        self.maxRef = maxRef

class Config():
    def __init__(self): 
        self.lm = LargeMotor(OUTPUT_B)
        self.rm = LargeMotor(OUTPUT_C)


        self.lls = LightSensor(INPUT_2)
        self.rls = LightSensor(INPUT_3)

        self.steering = MoveSteering(OUTPUT_B, OUTPUT_C)
        self.movetank = MoveTank(OUTPUT_B, OUTPUT_C)

class MotCtrl(Config):
    def __init__(self):
        
        #threading
        self.rlslock = threading.Lock()
        self.llslock = threading.Lock()
        self.pid = threading.Event()
        self.poll = threading.Event()
        
        #logging
        self.ls_log = []

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
                    override_intersection = False):
        
        if right_light_sensor:
            ls = self.rls
            acquired = self.rlslock.acquire()
        else:
            ls = self.lls
            acquired = self.llslock.acquire()
        
        ls.mode = 'COL-REFLECT'

        last_error = error = integral = 0.0
        derivative = 0.0
        
        while pid_run.is_set():

            print('ref:{ref}'.format(ref = ls.reflected_light_intensity))
            error = target_light_intensity - (100 * ( ls.reflected_light_intensity - ls.minRef ) / ( ls.maxRef - ls.minRef))

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
        
        acquired.release()
        

    def runpid(self,
                kp, 
                ki,
                kd,
                speed, 
                Daemon = None,
                target_light_intensity = None, 
                follow_left_edge = True, 
                right_light_sensor = True,
                poll_intersection = True):

        self.pid.set()
        self._pid = threading.Thread(target = self.pid_follow, args = (kp,ki,kd,speed), kwargs = {'target_light_intensity' : target_light_intensity, 'follow_left_edge' : follow_left_edge, 'right_light_sensor' : right_light_sensor, 'override_intersection' : intersection, 'pid_run' : self.pid}, daemon = Daemon)
        self._pid.start()

        if poll_intersection:
            start_inters_poll(not right_light_sensor)
    
    def start_inters_poll(self, right_light_sensor):
        self.poll.set()
        self._poll = threading.Thread(target = self.inters_pull, args = (right_light_sensor), kwargs = {'poll_run': self.poll})
        self._poll.start()
    
    def inters_poll(self, right_light_sensor, poll_run = True):
        
        if right_light_sensor:
            if self.rlslock.acquire(blocking = False): ls = self.rls 
            else: raise SensorUnavailable       
        else: 
            if self.llslock.acquire(): ls = self.lls
            else: raise SensorUnavailable
 
        ls.mode = 'COL-COLOR'

        while poll_run.is_set():
            time.sleep(0.2)
            self.ls_log.append(ls.color)

        acquire.release()
    
    def stop_poll(self):
        self.poll.clear()
        self._poll.join()
        
    def stop_pid(self):
        self.pid.clear()
        self._pid.join()
    
    def clear_log(self):
        self.ls_log = []

#Test
exe = MotCtrl()

exe.rls.calibrate_ref(0, 100)
exe.lls.calibrate_ref(0, 100)

exe.runpid(0.6,0,1,40,target_light_intensity = 20, right_light_sensor = True, poll_intersection = True)


while not exe.lm.position > 3000:
    print(exe.lm.position)   
exe.stop_pid()
   
