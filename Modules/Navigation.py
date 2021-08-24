#!/usr/bin/env python3

from ev3dev2.motor import OUTPUT_B, OUTPUT_C, MoveTank, MoveSteering, LargeMotor, SpeedPercent
from ev3dev2.led import Leds
from ev3dev2.button import Button
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.sensor import INPUT_2, INPUT_3
from ev3dev2.sound import Sound
import time
import threading
import collections

# Instantiate Brick
leds = Leds()
buttons = Button()
spk = Sound()

# Instantiate Sensors
left_ls = ColorSensor(INPUT_2)
right_ls = ColorSensor(INPUT_3)

minRef = 0 
maxRef = 100

# Instantiate Motors
left_motor = LargeMotor(OUTPUT_B)
right_motor = LargeMotor(OUTPUT_C)

steering = MoveSteering(OUTPUT_B, OUTPUT_C)
movetank = MoveTank(OUTPUT_B, OUTPUT_C)

# Program Ready
leds.set_color('LEFT', 'RED')
leds.set_color('RIGHT', 'RED')
while not buttons.enter:
    pass

class Navigate:
    def __init__(self):
        # Motors
        self.lm = left_motor
        self.rm = right_motor

        self.steering = steering
        self.movetank = movetank

        # Sensors
        self.lls = left_ls
        self.rls = right_ls
    
        # Threading
        self.pid = threading.Event()
        self.poll = threading.Event()
        
        # Logging
        self.ls_log = []
        self.intersection = 0
        self.intersection_log = []


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
        cycle = 0

        while pid_run.is_set():

            reflected_light_intensity = (100 * ( ls.reflected_light_intensity - minRef ) / ( maxRef - minRef))
            error = target_light_intensity - reflected_light_intensity

            if error == 0:
                integral = 0
            else:
                integral = integral + error
            
            if not cycle == 0:
                derivative = error - last_error
            last_error = error

            steering = (kp * error) + (ki * integral) + (kd * derivative)
            steering *= polarity
            
            self.steering.on(steering, SpeedPercent(speed))
            cycle += 1

            #print('cycle:{c}, ref:{ref}, error:{err}, steering:{steer} '.format(c = cycle, ref = reflected_light_intensity, steer = steering, err = error))
        
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
        q = collections.deque(maxlen = 3)

        while poll_run.is_set():
            #self.ls_log.append(ls.color_name)
            
            if ls.color != p_color:
                self.ls_log.append([cycle_count, ls.color])
                p_color = ls.color
                q.appendleft(ls.color)
                print(q)
                  
            if list(q) == [6, 1, 6]:
                self.intersection += 1
                self.intersection_log.append((self.intersection, self.lm.position, self.rm.position))
                spk.play_note('A4', 0.05)    
                
            cycle_count+= 1
                
    def stop_poll(self):
        self.poll.clear()
        self._poll.join()
        
    def stop_pid(self):
        self.pid.clear()
        self._pid.join()
    
    def clear_log(self):
        self.ls_log = []
        self.intersection = 0
        self.intersection_log = []     

#Test
exe = Navigate() 

exe.runpid(0.08, 0, 1 ,40, target_light_intensity = 40, right_light_sensor = True, follow_right_edge=True)

exe.start_inters_poll(right_light_sensor=False)


while True:
    if exe.intersection == 1:
        break

exe.stop_pid()
exe.stop_poll()

print(exe.intersection_log) 
print(exe.ls_log)
