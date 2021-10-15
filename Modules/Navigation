#!/usr/bin/env python3
from ev3dev2.motor import OUTPUT_B, OUTPUT_C, MoveTank, MoveSteering, LargeMotor, SpeedPercent
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.sensor import INPUT_2, INPUT_3
import threading

class Navigate:
    # Motors
    def __init__(self):
        self.lm = LargeMotor(OUTPUT_B)
        self.rm = LargeMotor(OUTPUT_C)

        self.steering = MoveSteering(OUTPUT_B, OUTPUT_C)
        self.movetank = MoveTank(OUTPUT_B, OUTPUT_C)

        # Sensors
        self.lls = ColorSensor(INPUT_2)
        self.rls = ColorSensor(INPUT_3)

        self.minRef = 0
        self.maxRef = 55
        
        # Threading
        self.pid = threading.Event()
        
    
    def reset_motor(self, right_motor = True, left_motor = True):
        if right_motor:
            self.rm.position = 0
        if left_motor:
            self.lm.position = 0
        

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
            raise ValueError('Required value f+ target_light_intensity')
        
        if right_light_sensor: ls = self.rls
        else: ls = self.lls
        
        ls.mode = 'COL-REFLECT'

        if follow_right_edge: polarity = 1
        else: polarity = -1 
        
        last_error = error = integral = 0.0
        derivative = 0.0
        cycle = 0

        while pid_run.is_set():

            global minRef
            global maxRef
            
            reflected_light_intensity = (100 * ( ls.reflected_light_intensity - self.minRef ) / ( self.maxRef - self.minRef))
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
    
    def stop_pid(self):
        self.pid.clear()
        self._pid.join()

    def align(self):
        pass
    
