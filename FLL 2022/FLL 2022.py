# pyright: reportMissingImports=false

from spike import ColorSensor, MotionSensor, Motor, MotorPair, PrimeHub
import math
import time

# Hub
hub = PrimeHub()

# Motors
lm = Motor('A')
rm = Motor('B')

# Motor polarity
lm_polarity = -1
rm_polarity = 1

lm.set_stop_action('brake')
rm.set_stop_action('brake')

pair = MotorPair('B','A')
pair.set_motor_rotation(17.6 * math.pi, 'cm') # Sizes: Small (5.6cm) Large (17.6cm)


fmm = Motor('C')
bmm = Motor('D')

# Sensors
lls = ColorSensor('E')
rls = ColorSensor('F')

gyro = MotionSensor()

# Constants
minRef = 34
maxRef = 99

def lapse():
    global clock_start

    duration = round((time.ticks_ms() - clock_start)/1000, 2)
    clock_start = time.ticks_ms()
    return duration 

def print_timings():
    print(" run 1: {r1} \n Equip run 2: {e1}, run 2: {r2} \n Equip run 3: {e2}, run 3:{r3}".format(r1 = t_run1, r2 = t_run2, r3 = t_run3, e1 = t_equipr2, e2 = t_equipr3))

def reset_m():
    rm.set_degrees_counted(0)
    lm.set_degrees_counted(0)

# For testing
def pauseloop():
    hub.status_light.on('orange')
    while not hub.left_button.is_pressed() and not hub.right_button.is_pressed():
        pass
    hub.status_light.on('green')

# Recording runs 
def change_mech():

    # Ideal Time 
    i_run1 =  80
    i_run2 = 20
    i_run3 = 40

    hub.status_light.on('orange')
    while not hub.left_button.is_pressed() and not hub.right_button.is_pressed():
        pass
    
    # Custom to our runs
        
    hub.status_light.on('green')
    
def measure_m():

    reset_m()

    while not hub.left_button.is_pressed() and not hub.right_button.is_pressed():
        print("RM: {rm}, LM: {lm}".format(rm = rm.get_degrees_counted(), lm = lm.get_degrees_counted()))

    reset_m()
    measure_m()


def calibrate_ls():

    minRef = 0
    maxRef = 100
    dataleft = []
    dataright = []

    # State
    state = 1
    hub.light_matrix.write(str(state))

    while True:
        if hub.left_button.is_pressed() and state < 3:
            time.sleep_ms(500)
            state += 1
            hub.light_matrix.write(str(state))

        elif hub.right_button.is_pressed() and state > 1:
            state -= 1
            time.sleep_ms(500)
            hub.light_matrix.write(str(state))

        dataleft.append(lls.get_reflected_light())
        dataright.append(rls.get_reflected_light())
        minRef = (min(dataleft)+min(dataright))/2
        maxRef = (max(dataleft)+max(dataright))/2

        if state == 1:
            print('l.ref: {lref}, r.ref: {rref}'.format(lref = lls.get_reflected_light(),
                                                        rref = rls.get_reflected_light()))
        elif state == 2:
            print('l.ref: {lref}, r.ref: {rref}'.format(lref = (100 * (lls.get_reflected_light() - minRef ) / ( maxRef - minRef)),
                                                        rref = (100 * (rls.get_reflected_light() - minRef ) / ( maxRef - minRef))))

        elif state == 3:
            print('minref: {minref}, maxref: {maxref}'.format(maxref = maxRef,
                                                            minref = minRef))


def line_follow(speed,
                deg,
                target = None,
                right_edge = True,
                right_ls = True,
                forward = True,
                scale_ref = False,
                track_right = True):

    if target is None: raise ValueError('Missing Light Value')

    if right_ls: ls = rls
    else: ls = lls
    if right_edge: polarity = -1
    else: polarity = 1

    # Our robot forward is -ve
    speed *= -1

    if track_right: tracked_motor = rm
    else: tracked_motor = lm
    if not forward:
        speed *= -1
        polarity *= 1

    last_error = error = integral = 0.0
    derivative = 0.0

    reset_m()

    while not abs(tracked_motor.get_degrees_counted()) > deg:
        if scale_ref:
            reflected_light_intensity = (100 * (ls.get_reflected_light() - minRef ) / ( maxRef - minRef ))
        else:
            reflected_light_intensity = ls.get_reflected_light()
        error = target - reflected_light_intensity

        if error == 0:
            integral = 0
        else:
            integral = integral + error

        derivative = error - last_error
        last_error = error

        steering = round((lkp * error) + (lki * integral) + (lkd * derivative))
        steering *= polarity

        pair.start_at_power(speed, steering = steering)

        # print('ref:{ref}, error:{err}, steering:{steer} '.format(ref = reflected_light_intensity, steer = steering, err = error))
    pair.stop()

def line_align(start_spd,
        maxduration,
        marginoferror,
        target = None,
        black_white = True,
        correction_spd = None,
        pos_beforeline = True,
        rel_front_back_line = True,
        timeout = True):

    if pos_beforeline: polarity = 1
    else: polarity = -1

    if correction_spd is None: correction_spd = start_spd

    # Our robot forward is -ve
    start_spd *= -1
    correction_spd *= -1

    if target is None: raise ValueError('Missing Light Value')
    else:
        rls_target_light_intensity = target
        lls_target_light_intensity = target


    rls_target_black = rls_target_light_intensity - marginoferror
    rls_target_white = rls_target_light_intensity + marginoferror
    lls_target_black = lls_target_light_intensity - marginoferror
    lls_target_white = lls_target_light_intensity + marginoferror

    start_spd = start_spd*polarity

    if rel_front_back_line:
        forward_spd = 1 * correction_spd*polarity
        backward_spd = -1 * correction_spd*polarity
    else:
        forward_spd = -1 * correction_spd*polarity
        backward_spd = 1 * correction_spd*polarity

    # Coarse Adjustment

    while True:
        pair.start(steering = 0, speed = start_spd)

        if black_white:

            # Aligning towards black Line

            if lls.get_reflected_light() <= lls_target_light_intensity:
                lm.stop()
                while not rls.get_reflected_light() <= rls_target_light_intensity:
                    rm.start(speed = correction_spd * -rm_polarity)
                    # print(rls.get_reflected_light())
                rm.stop()
                break

            else:
                if rls.get_reflected_light() <= rls_target_light_intensity:
                    rm.stop()
                    while not lls.get_reflected_light() <= lls_target_light_intensity:
                        lm.start(speed = correction_spd * -lm_polarity)
                        # print(lls.get_reflected_light())
                    lm.stop()
                    break
                else:
                    pass

        else:

            # Aligning towards White Line

            if lls.get_reflected_light() >= lls_target_light_intensity:
                lm.stop()
                while not rls.get_reflected_light() >= rls_target_light_intensity:
                    rm.start(speed = correction_spd * rm_polarity)
                    # print(rls.get_reflected_light())
                rm.stop()
                break

            else:
                if rls.get_reflected_light() >= rls_target_light_intensity:
                    rm.stop()
                    while not lls.get_reflected_light() >= lls_target_light_intensity:
                        lm.start(speed = correction_spd * lm_polarity)
                        # print(lls.get_reflected_light())
                    lm.stop()
                    break
                else:
                    pass

    # Fine Adjustment
    start = time.ticks_ms()
    end = time.ticks_ms()


    while not (end - start) > maxduration*1000:

        if lls.get_reflected_light() < lls_target_light_intensity:
            lm.start(speed = forward_spd * lm_polarity)
        else:
            if lls.get_reflected_light() > lls_target_light_intensity:
                lm.start(speed = backward_spd * lm_polarity)

            else:
                lm.stop()

        if rls.get_reflected_light() < rls_target_light_intensity:
            rm.start(speed = forward_spd * rm_polarity)
        else:
            if rls.get_reflected_light() > rls_target_light_intensity:
                rm.start(speed = backward_spd * rm_polarity)

            else:
                rm.stop()

        if lls_target_black < lls.get_reflected_light() < lls_target_white and rls_target_black < rls.get_reflected_light() < rls_target_white:
            break

        if timeout:
            end = time.ticks_ms()
        else:
            end = start

    rm.stop()
    lm.stop()

    # print(lls.get_reflected_light())
    # print(rls.get_reflected_light())

def gyro_straight(speed,
                deg,
                target = None,
                track_right = True,
                forward = True):

    # Our robot forward is -ve
    speed *= -1

    # -ve steering: right, +ve steering: left
    polarity = -1

    if not forward:
        polarity *= -1
        speed *= -1 
    if target is None: raise ValueError('Missing Gyro Value')
    last_error = error = integral = 0.0
    derivative = 0.0

    if track_right: tracked_motor = rm
    else: tracked_motor = lm

    reset_m()

    while not abs(tracked_motor.get_degrees_counted()) > deg:
        o_yaw = gyro.get_yaw_angle()
        error = target - o_yaw

        if error == 0:
            integral = 0
        else:
            integral = integral + error

        derivative = error - last_error
        last_error = error

        steering = ((gkp * error) + (gki * integral) + (gkd * derivative))*polarity

        pair.start(steering = round(steering), speed = speed)
        # print('reading: {r}, error: {e}, steering: {s}'.format(r= o_yaw, e = error, s = steering))

    pair.stop()

def gyro_turn(target, marginoferror, speed, timeout = False, steering = 100):
    start = time.ticks_ms()
    end = time.ticks_ms()

    if type(timeout) == 'int': sec = timeout
    else: sec = 0

    while not (end-start) > sec*1000:
        o_yaw = gyro.get_yaw_angle()
        if target - o_yaw < 0:
            pair.start(speed = speed, steering = steering * -1)
        elif target - o_yaw > 0:
            pair.start(speed = speed, steering = steering)
        elif abs(target - o_yaw) <= marginoferror:
            break

        if timeout:
            end = time.ticks_ms()
            print('timeout')

    lm.stop()
    rm.stop()

def run_1(run):

    if not run:
        return

    # Run 1 starts
    gyro.reset_yaw_angle()
    reset_m()
    bmm.run_for_degrees(160, -30)

    # Move out of Base
    gyro_straight(30, 500, target = 0)
    gyro_turn(82, 0,20)
    gyro_straight(50, 1950, target = 85)

    # M14: Bridge
    gyro_turn(107, 1,10)
    gyro_straight(40, 200, target = 108)
    bmm.run_for_degrees(50, 30)

    # M11: Home Delivery
    gyro_straight(60, 580, target = 108)

    # M14: Bridge
    bmm.run_for_degrees(50, -30)
    pair.move(550, unit = 'degrees', steering = 0, speed = 70)
    bmm.run_for_degrees(50, 30)

    # Transition
    gyro_turn(55, 1,15)
    gyro_straight(40, 1135, target = 55)

    # M10: Sorting Center
    gyro_turn(167, 1,20)
    pair.move(180, unit = 'degrees', steering = 0, speed = 50)
    # fmm.run_for_seconds(1.4, speed = -100)
    gyro_straight(50, 290, target = 170)
    # fmm.run_for_seconds(1.1, speed = 100)

    # M09: Joining Track Tracks
    pair.move(200, unit = 'degrees', steering = 0, speed = 50)
    rm.start(speed = -35)
    # fmm.start(speed = 100)
    time.sleep(0.65)
    while not gyro.get_yaw_angle() > -5:
        pass
    fmm.stop()
    pair.stop()
    pair.move(150, unit = 'degrees', steering = 0, speed = 50)
    pair.move(20, unit = 'degrees', steering = 100, speed = -20)
    bmm.start(speed = 30)
    time.sleep(0.6)
    bmm.stop()
    pair.move(80, unit = 'degrees', steering = -100, speed = -40)

    # Transition
    bmm.run_for_degrees(60, -20)
    pair.move(50, unit = 'degrees', steering = -100, speed = 40)
    line_follow(40, 270, right_edge = False, right_ls = True, target = 62, scale_ref = False)
    gyro_turn(-21, 1,20)
    pair.move(660, unit = 'degrees', steering = 0, speed = -30)
    gyro_turn(-8, 1,20)

    # M09: Push train
    bmm.set_stop_action('brake')
    bmm.run_for_seconds(0.4, 50)
    gyro_straight(35, 950, target = -8, forward = False)

    #Transition
    pair.move(50, unit = 'degrees', steering = 0, speed = -30)
    bmm.run_for_degrees(-80, 30)
    gyro_straight(50, 350, target = -3)
    line_align(35,1.5,2,target = 62,correction_spd = 15, pos_beforeline = True, rel_front_back_line = False, timeout = True)
    gyro.reset_yaw_angle()

    # M08: Air Drop
    pair.start(steering = 96, speed = -30)
    while not gyro.get_yaw_angle() < -100:
        pass
    pair.stop()
    pair.move(550, unit = 'degrees', steering = 0, speed = 50)

    # Transition
    line_follow(50, 770, right_edge = False, right_ls = False, target = 62, scale_ref = False)
    gyro_turn(-62, 1,15)
    gyro_straight(50, 550, target = -62)

    # Crane
    bmm.run_for_seconds(0.4, 60)
    pair.move(350, unit = 'degrees', steering = 25, speed = 40)
    pair.move(100, unit = 'degrees', steering = 0, speed = -40)
    bmm.run_for_degrees(80, -30)

    # Transition
    gyro_straight(50, 280, target = -62)
    line_align(50,0.5,2,target = 62,correction_spd = 15, pos_beforeline = True, rel_front_back_line = True, timeout = True)
    gyro.reset_yaw_angle()

    # Eject
    gyro_straight(40, 950, target = -6)
    bmm.run_for_seconds(0.25, 80)
    gyro_turn(-17, 1,20)
    pair.move(330, unit = 'degrees', steering = 0, speed = -70)
    gyro_turn(0, 1,20)
    pair.move(120, unit = 'degrees', steering = 0, speed = 70)
    bmm.run_for_degrees(100, -50)

    # Exit Run 1
    fmm.start(speed = -100)
    gyro_straight(100, 1150, target = -3)
    fmm.stop()

def run_2(run):

    if not run:
        return 

    # Run 2 starts
    gyro.reset_yaw_angle()

    # Mission 1
    gyro_straight(80, 700, target = 0)
    
    line_follow(40, 730, right_edge = False, right_ls = False, target = 62, scale_ref = False)
    pair.move(30, unit = 'degrees', steering = 0, speed = -15)
    pair.move(20, unit = 'degrees', steering = 0, speed = 15)
    fmm.run_for_degrees(100, 100)

    # Mission 2
    pair.move(150, unit = 'degrees', steering = 0, speed = 30)
    gyro_turn(-42, 1,15)
    fmm.run_for_degrees(100, -100)

    pair.move(120, unit = 'degrees', steering = 0, speed = 30)
    bmm.run_for_degrees(100, speed = 100)
    fmm.run_for_degrees(120, 100)
    pair.move(370, unit = 'degrees', steering = 0, speed = -30)
    bmm.run_for_degrees(100, speed = -100)
    pair.move(120, unit = 'degrees', steering = 0, speed = 30)
    lm.run_for_degrees(340, speed = -30 * lm_polarity)
    bmm.run_for_degrees(100, speed = 100)
    pair.move(200, unit = 'degrees', steering = 0, speed = -30)
    bmm.run_for_degrees(100, speed = -100)
    pair.move(1200, unit = 'degrees', steering = 0, speed = -100)

    # Return to base


def run_3(run):

    if not run:
        return  

    # Run 3 starts
    gyro.reset_yaw_angle()
    reset_m()

    # Move out of Base
    gyro_straight(50, 1100, target = 0, forward = False)
    pair.start(steering = -43, speed = -30)
    while not gyro.get_yaw_angle() > 38:
        pass
    lm.stop()

    # Platoon Truck
    bmm.start(speed = -55)
    pair.move(200, unit = 'degrees', steering = 0, speed = -50)
    bmm.stop()
    gyro_straight(20, 550, target = 40, forward = False)
    bmm.start(speed = 100)
    time.sleep(0.5)
    bmm.stop()
    pair.move(150, unit = 'degrees', steering = 0, speed = -20)

    # Transition
    rm.start(speed = 50*rm_polarity)
    while not gyro.get_yaw_angle() < 8:
        pass
    rm.stop()
    bmm.stop()
    pair.move(400, unit = 'degrees', steering = 0, speed = 50)
    rm.start(speed = -40*rm_polarity)
    while not gyro.get_yaw_angle() > 35:
        pass
    rm.stop()

    # Creative Project
    gyro_straight(50, 1320, target = 42, forward = False)
    gyro_turn(-68, 1,15)
    pair.move(180, unit = 'degrees', steering = 0, speed = -50)
    
    fmm.start(speed = 100)
    time.sleep(0.3)
    fmm.stop()

    # Transition
    pair.start(steering = 0, speed = 40)
    while not rls.get_reflected_light() < 50: 
        pass
    pair.stop()
    line_align(40,1.5,2,target = 62,correction_spd = 15, pos_beforeline = False, rel_front_back_line = True, timeout = True)
    gyro.reset_yaw_angle()
    pair.move(20, unit = 'degrees', steering = 0, speed = -10)
    lm.start(speed = 30*lm_polarity)
    while not gyro.get_yaw_angle() > 88:
        pass
    lm.stop()
    gyro_straight(60, 880, target = 88, forward = False)

    # Blue Cargo
    lm.start(speed = 50*lm_polarity)
    rm.start(speed = -20*rm_polarity)
    while not gyro.get_yaw_angle() > 177:
        pass
    lm.stop()
    rm.stop()
    
    fmm.start(speed = -100)
    time.sleep(0.45)
    fmm.stop()

    # Transition
    lm.start(speed = -50*lm_polarity)
    rm.start(speed = 20*rm_polarity)
    time.sleep(0.80)
    lm.stop()
    rm.stop()

    # Grey Cargo
    rm.start(speed = 50*rm_polarity)
    lm.start(speed = -28*lm_polarity)
    while not gyro.get_yaw_angle() < 20:
        pass
    rm.stop()
    lm.stop()
    fmm.start(speed = 100)
    time.sleep(0.45)
    fmm.stop()
    
    # Transition
    rm.start(speed = -50*rm_polarity)
    lm.start(speed = 28*lm_polarity)
    while not gyro.get_yaw_angle() > 84:
        pass
    rm.stop()
    lm.stop()
    if lls.get_reflected_light() > rls.get_reflected_light():
        line_follow(50, 900, right_edge = True, right_ls = True, target = 62, scale_ref = False)
    else:
        line_follow(50, 900, right_edge = False, right_ls = False, target = 62, scale_ref = False, track_right = False)

    lm.start(speed = 50 * lm_polarity)
    while not gyro.get_yaw_angle() > 170:
        pass
    lm.stop()
    gyro_straight(70, 920, target = 170, forward = True)
    lm.start(speed = 8*lm_polarity)
    rm.start(speed = 40*rm_polarity)
    while not gyro.get_yaw_angle() < 118:
        pass
    lm.stop()
    rm.stop()

    # Unloading Cargo
    gyro_straight(30, 250, target = 118, forward = False)
    bmm.start(speed =  -30)
    time.sleep(0.7)

    # Accident Avoidance 
    pair.start(steering = 0, speed = -30)
    while not rls.get_reflected_light() < 62:
        pass
    pair.stop()
    rm.run_for_degrees(60, speed =  30*rm_polarity)

# Time Recording
t_run1 = 0
t_run2 = 0
t_run3 = 0

t_equipr2 = 0
t_equipr3 = 0


## Run 1 
# Run 1 Constants
gkp = 2
gki = 0
gkd = 0.2

lkp = 1.65
lki = 0
lkd = 0

# Motor Defaults
lm.set_stop_action('brake')
rm.set_stop_action('brake')
fmm.set_stop_action('hold')
bmm.set_stop_action('hold')

# Run 1 Execution
pauseloop()
clock_start = time.ticks_ms()
run_1(True)
t_run1 = lapse()

## Run 2 
# Run 2 Constants
gkp = 3
gki = 0
gkd = 0.5

lkp = 1.65
lki = 0
lkd = 0

# Run 2 Motor Defaults
lm.set_stop_action('brake')
rm.set_stop_action('brake')
fmm.set_stop_action('brake')
bmm.set_stop_action('brake')

# Run 2 Execution
pauseloop()
run_2(True)
t_run2 = lapse()

## Run 3
# Run 3 Constants
gkp = 2
gki = 0
gkd = 0.2

lkp = 1.3
lki = 0
lkd = 0

# Run 3 Motor Defaults 
lm.set_stop_action('brake')
rm.set_stop_action('brake')
fmm.set_stop_action('hold')
bmm.set_stop_action('brake')

# Run 3 Execution
pauseloop()
run_3(True)
t_run3 = lapse()

# print_timings()

raise SystemExit

