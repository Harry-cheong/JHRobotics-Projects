#!/usr/bin/env python3

from ev3dev2.motor import OUTPUT_B, OUTPUT_C, MoveTank, MoveSteering, LargeMotor, SpeedPercent
from ev3dev2.led import Leds
from ev3dev2.button import Button
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.sensor import INPUT_2, INPUT_3
from time import sleep
import threading
import collections

#Sensors and Brick
leds = Leds()
buttons = Button()

run = threading.Event()

#Motors
tank = MoveTank(OUTPUT_C,OUTPUT_B)

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

    def calibrate_ref(self, minRef, maxRef):
        self.minRef = minRef
        self.maxRef = maxRef

class MotCtrl(Config):
    def __init__(self):
    
        #threading
        self.pid = threading.Event()
        self.poll = threading.Event()
        
        #logging
        self.ls_log = []
        self.analyse_log = []
        self.intersection = False

        #inheritance 
        Config.__init__(self)

    def pid_follow(self,
                    kp,
                    ki,
                    kd,
                    speed,
                    pid_run = True,
                    target_light_intensity = None,
                    follow_right_edge = True,
                    right_light_sensor = True):
        
        if target_light_intensity is None:
            raise ValueError('Required value for **kwargs target_light_intensity')
        
        if right_light_sensor: ls = self.rls
        else: ls = self.lls
        
        ls.mode = 'COL-REFLECT'

        if follow_right_edge: polarity = 1
        else: polarity = -1 
        
        last_error = error = integral = 0.0
        derivative = 0.0
        
        while pid_run.is_set():
            reflected_light_intensity = (100 * ( ls.reflected_light_intensity - self.minRef ) / ( self.maxRef - self.minRef))
            error = target_light_intensity - reflected_light_intensity

            if error == 0:
                integral = 0
            else:
                integral = integral + error

            derivative = error - last_error
            last_error = error

            steering = (kp * error) + (ki * integral) + (kd * derivative)
            steering *= polarity

            self.steering.on(steering, SpeedPercent(speed))
            #print('ref:{ref}, derivative:{d}, steering:{steer} '.format(ref = reflected_light_intensity, steer = steering, d = derivative))
        
        self.movetank.off()
        
        

    def runpid(self,
                kp, 
                ki,
                kd,
                speed, 
                Daemon = None,
                target_light_intensity = None, 
                follow_right_edge = True, 
                right_light_sensor = True,
                poll_intersection = True):

        self.intersection = False
        self.pid.set()
        self._pid = threading.Thread(target = self.pid_follow, args = (kp,ki,kd,speed), kwargs = {'target_light_intensity' : target_light_intensity, 'follow_right_edge' : follow_right_edge, 'right_light_sensor' : right_light_sensor, 'pid_run' : self.pid}, daemon = Daemon)
        self._pid.start()
    
    def start_inters_poll(self, right_light_sensor = False):
        self.poll.set()
        self._poll = threading.Thread(target = self.inters_poll, kwargs = {'right_light_sensor': right_light_sensor,'poll_run': self.poll})
        self._poll.start()
    
    def inters_poll(self, right_light_sensor, poll_run = True):
        
        if right_light_sensor:
            ls = self.rls       
        else: 
            ls = self.lls
 
        ls.mode = 'COL-COLOR'
        p_color = 0
        cycle_count = 0
        q =collections.deque(maxlen = 3)

        while poll_run.is_set():
            self.ls_log.append(ls.color)
            cycle_count+= 1

            if ls.color != p_color:
                try:
                    append_count = cycle_count - self.analyse_log[-1][0] 
                except:
                    append_count = cycle_count
                self.analyse_log.append([cycle_count, append_count, ls.color_name])
                p_color = ls.color
                q.appendleft(ls.color)
                #print(q)
                  
            if list(q) == [6, 1, 6]:
                self.intersection = True
                
    
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

exe.calibrate_ref(0, 55)

exe.runpid(0.06, 0, 0.8 ,40,target_light_intensity = 35, right_light_sensor = True,follow_right_edge=True)
exe.start_inters_poll(right_light_sensor=False)
#0.23, 0, 0.42, 40

while not exe.lm.position > 2200:
    if exe.intersection is True:
        break

exe.stop_poll()   
print(exe.analyse_log) 
exe.stop_pid()
