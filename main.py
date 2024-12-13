# -*- coding: utf-8 -*-

from pyb import Pin, Timer, ExtInt
from array import array # array left in case we used later
import Motor_Driver
import Encoder_Driver
import BNO055_Driver
import linesensor_driver
import gc
import cotask
import task_share
import cqueue
import time

# Task for Motor A and Encoder A
def mot_enc_task_A():
    global mot_A, enc_A # globals needed
    mot_A.enable()
    while True:
        mot_A.set_duty(duty_A_share.get()) # set duty of Motor A to share duty var
        enc_A.update() # update encoder A values
        pos_A_share.put(enc_A.get_position()) # put position A into coms queue
        delta_A_share.put(enc_A.get_delta()) # put delta A into coms queue
        vel_A = enc_A.get_velocity()[0] # get velocity from A
        ticks_A = enc_A.get_velocity()[1] # get ticks from A
        velm_A_share.put(vel_A) # put velocity A into measured velocity share var
        ticks_A_share.put(ticks_A) # put ticks A into ticks share var
        yield 0

# Task for Motor B and Encoder B
def mot_enc_task_B():
    global mot_B, enc_B # globals needed
    mot_B.enable()
    while True:
        mot_B.set_duty(duty_B_share.get()) # set duty of Motor B to share duty var
        enc_B.update() # update encoder B values
        pos_B_share.put(enc_B.get_position()) # put position Binto coms queue
        delta_B_share.put(enc_B.get_delta()) #put delta B into coms queue
        vel_B = enc_B.get_velocity()[0] # get velocity from B
        ticks_B = enc_B.get_velocity()[1] # get ticks from B 
        velm_B_share.put(vel_B) # put velocity B into measured velocity share var
        ticks_B_share.put(ticks_B) # put ticks B into ticks share var
        yield 0

# def abso_position():
#     global enc_A, enc_B
#     theta = 0
#     while True:
#         wheel_rad = 0.035
#         track_width = 0.141
#         A_pos = pos_A_share.get()
#         B_pos = pos_B_share.get()
#         A_dist = A_pos*wheel_rad
#         B_dist = B_pos*wheel_rad
#         d_theta = (B_dist-A_dist)/track_width
#         theta += d_theta
#         theta_share.put(theta)
#         yield 0
        
# Task for IMU reading
def read_imu_task():
    global imu # globals needed
    while True:
        # Read IMU data
        imu.read_eula() # create tuple of euler angle data
        imu.read_angv() # create tuple of angular velocity data
        eula_x = imu.read_eula()[0] # put euler x (yaw) into eula_x var
        eula_x_share.put(eula_x) # put euler x into shared var
        # BELOW CODE COMMENTED OUT DUE TO ONLY USING EULER X CURRENTLY
        yield 0

# Task to read line sensors
def read_ls_task():
    global ls
    path_end = 0
    while True:
        bump_end = bump_end_share.get()
        sensor_1_raw = ls.measure_reflectivity(ls.ls_1)
        sensor_2_raw = ls.measure_reflectivity(ls.ls_2)
        sensor_3_raw = ls.measure_reflectivity(ls.ls_3)
        sensor_4_raw = ls.measure_reflectivity(ls.ls_4)
        sensor_5_raw = ls.measure_reflectivity(ls.ls_5)
        sensor_6_raw = ls.measure_reflectivity(ls.ls_6)
        sensor_7_raw = ls.measure_reflectivity(ls.ls_7)
        sensor_8_raw = ls.measure_reflectivity(ls.ls_8)
        if sensor_1_raw > 1000:
            sensor_1 = 1
        else:
            sensor_1 = 0
        if sensor_2_raw > 1000:
            sensor_2 = 1
        else:
            sensor_2 = 0
        if sensor_3_raw > 1000:
            sensor_3 = 1
        else:
            sensor_3 = 0
        if sensor_4_raw > 1000:
            sensor_4 = 1
        else:
            sensor_4 = 0
        if sensor_5_raw > 1000:
            sensor_5 = 1
        else:
            sensor_5 = 0
        if sensor_6_raw > 1000:
            sensor_6 = 1
        else:
            sensor_6 = 0
        if sensor_7_raw > 1000:
            sensor_7 = 1
        else:
            sensor_7 = 0
        if sensor_8_raw > 1000:
            sensor_8 = 1
        else:
            sensor_8 = 0
        if sensor_1 == sensor_2 == sensor_3 == sensor_4 == sensor_5 == sensor_6 == sensor_7 == sensor_8 == 1 & bump_end == 1:
            path_end += 1
            end_share.put(path_end)
            bump_end_share.put(0)
        try:
            d = ((0.5*sensor_5+1.5*sensor_6+2.5*sensor_7+3.5*sensor_8)-(3.5*sensor_1+2.5*sensor_2+1.5*sensor_3+0.5*sensor_4))/(sensor_5+sensor_6+sensor_7+sensor_8+sensor_1+sensor_2+sensor_3+sensor_4)
            d_share.put(-d)
        except ZeroDivisionError:
            d_share.put(0)
        yield 0

def bump_task():
    global bump_pins
    # Interrupt handler for bump sensor pins
    def bumped(pin):
        try:
            bump_share.put(1)  # Set bump detected flag
        except Exception as e:
            print(f"Error in bump handler: {e}")
    # Attach the interrupt to all bump pins
    for bump_pin in bump_pins:
        bump_pin.irq(trigger=Pin.IRQ_FALLING, handler=bumped)

    while True:
        yield 0

def control_task(vel_ref):
    global Kp, enc_A, enc_B, Kd, Ki
    buf = 0
    tracker = 0
    while True:
        eula_x = eula_x_share.get()
        bump = bump_share.get()
        velm_A = velm_A_share.get()
        dticks_A = ticks_A_share.get()
        velm_B = velm_B_share.get()
        dticks_B = ticks_B_share.get()
        error_A = vel_ref-velm_A
        error_B = vel_ref-velm_B
        err_A_share.put(error_A)
        err_B_share.put(error_B)
        d = d_share.get()
        end = end_share.get()
        correction_A = 50*d
        correction_B = 50*d
        if end == 0:
            if bump==0:
                if d >= 0:
                    control_A_input = (error_A*Kp)+(Kd*error_A/(dticks_A/1000000))+(Ki*error_A*dticks_A/1000000)
                    control_B_input = (error_B*Kp)+(Kd*error_B/(dticks_B/1000000))+(Ki*error_B*dticks_B/1000000)-correction_B
                else:
                    control_A_input = (error_A*Kp)+(Kd*error_A/(dticks_A/1000000))+(Ki*error_A*dticks_A/1000000)+correction_A
                    control_B_input = (error_B*Kp)+(Kd*error_B/(dticks_B/1000000))+(Ki*error_B*dticks_B/1000000)
                duty_A_input = max(-100, min(100, control_A_input)) # ensure control loop duty is less than 100 absolute value
                duty_A_share.put(duty_A_input) # put input into duty A share
                duty_B_input = max(-100, min(100, control_B_input)) # ensure control loop duty is less than 100 absolute value
                duty_B_share.put(duty_B_input) # put input into duty B share
            if bump==1:
                if tracker==0:
                    vel_refA = -20
                    vel_refB = -20
                    buf += 1
                    if buf >= 20:
                        tracker += 1
                        buf = 0
                if tracker==1:
                    vel_refA = 10
                    vel_refB = -10
                    buf += 1
                    if buf >= 25:
                        tracker += 1
                        buf = 0
                if tracker==2:
                    vel_refA = 20
                    vel_refB = 20
                    buf += 1
                    if buf >= 50:
                        tracker += 1
                        buf = 0
                if tracker==3:
                    vel_refA = -10
                    vel_refB = 10
                    buf += 1
                    if buf >= 25:
                        tracker += 1
                        buf = 0          
                if tracker==4:
                    vel_refA = 20
                    vel_refB = 20
                    buf += 1
                    if buf >= 40:
                        tracker += 1
                        buf = 0  
                if tracker==5:
                    vel_refA = 12
                    vel_refB = 25
                    buf += 1
                    if buf >= 25:
                        if d!= 0:
                            tracker += 1
                            buf = 0
                if tracker==6:
                    vel_refA = 20
                    vel_refB = -20
                    if eula_x < 17 or eula_x > 345:
                        bump_share.put(0)
                        tracker = 0
                        bump_end_share.put(1)
                error_A1 = vel_refA-velm_A
                error_B1 = vel_refB-velm_B
                control_A_input = (error_A1*Kp)+(Kd*error_A1/(dticks_A/1000000))+(Ki*error_A1*dticks_A/1000000)
                control_B_input = (error_B1*Kp)+(Kd*error_B1/(dticks_B/1000000))+(Ki*error_B1*dticks_B/1000000)
                duty_A_input = max(-100, min(100, control_A_input)) # ensure control loop duty is less than 100 absolute value
                duty_A_share.put(duty_A_input) # put input into duty A share
                duty_B_input = max(-100, min(100, control_B_input)) # ensure control loop duty is less than 100 absolute value
                duty_B_share.put(duty_B_input) # put input into duty B share
        if end == 1:
            if tracker == 0:
                vel_refA = 20
                vel_refB = 20
                error_A1 = vel_refA-velm_A
                error_B1 = vel_refB-velm_B
                control_A_input = (error_A1*Kp)+(Kd*error_A1/(dticks_A/1000000))+(Ki*error_A1*dticks_A/1000000)
                control_B_input = (error_B1*Kp)+(Kd*error_B1/(dticks_B/1000000))+(Ki*error_B1*dticks_B/1000000)
                duty_A_input = max(-100, min(100, control_A_input)) # ensure control loop duty is less than 100 absolute value
                duty_A_share.put(duty_A_input) # put input into duty A share
                duty_B_input = max(-100, min(100, control_B_input)) # ensure control loop duty is less than 100 absolute value
                duty_B_share.put(duty_B_input) # put input into duty B share
                buf += 1
                if buf >= 20:
                    #duty_A_share.put(0)
                    #duty_A_share.put(0)
                    buf = 0
                    tracker += 1
            if tracker == 1:
                duty_A_share.put(0)
                duty_B_share.put(0)
                buf += 1
                if buf >= 40:
                    buf = 0
                    tracker += 1
            if tracker == 2:
                if eula_x <= 270:
                    eu_correction_A = 0.3*(-30-eula_x)
                    eu_correction_B = 0.3*(-30-eula_x)
                    control_A_input = eu_correction_A
                    control_B_input = -eu_correction_B
                else:
                    eu_correction_A = 0.3*(350-eula_x)
                    eu_correction_B = 0.3*(350-eula_x)
                    control_A_input = -eu_correction_A
                    control_B_input = eu_correction_B
                duty_A_input = max(-100, min(100, control_A_input)) # ensure control loop duty is less than 100 absolute value
                duty_A_share.put(duty_A_input) # put input into duty A share
                duty_B_input = max(-100, min(100, control_B_input)) # ensure control loop duty is less than 100 absolute value
                duty_B_share.put(duty_B_input) # put input into duty B share
                buf += 1
                if buf >= 150:
                    tracker += 1
                    buf = 0
                #if eula_x < 24 or eula_x > 345:
                    #tracker += 1
            if tracker == 3:
                duty_A_share.put(0)
                duty_B_share.put(0)
                buf += 1
                if buf >= 20:
                    tracker += 1
                    buf = 0
            if tracker == 4:
                # Go straight until it sees a full line
                vel_refA = 20
                vel_refB = 20
                error_A1 = vel_refA-velm_A
                error_B1 = vel_refB-velm_B
                if eula_x <= 350:
                    control_A_input = (error_A1*Kp)+(Kd*error_A1/(dticks_A/1000000))+(Ki*error_A1*dticks_A/1000000)-(0.7*eula_x)
                    control_B_input = (error_B1*Kp)+(Kd*error_B1/(dticks_B/1000000))+(Ki*error_B1*dticks_B/1000000)
                else:
                    control_A_input = (error_A1*Kp)+(Kd*error_A1/(dticks_A/1000000))+(Ki*error_A1*dticks_A/1000000)
                    control_B_input = (error_B1*Kp)+(Kd*error_B1/(dticks_B/1000000))+(Ki*error_B1*dticks_B/1000000)-(0.7*(360-eula_x))
                duty_A_input = max(-100, min(100, control_A_input)) # ensure control loop duty is less than 100 absolute value
                duty_A_share.put(duty_A_input) # put input into duty A share
                duty_B_input = max(-100, min(100, control_B_input)) # ensure control loop duty is less than 100 absolute value
                duty_B_share.put(duty_B_input) # put input into duty B share
                buf+=1
                if buf >= 40:
                    bump_end_share.put(1)
                    buf = 0
                    tracker += 1
        if end == 2:
            if tracker == 5:
                vel_refA = 20
                vel_refB = 20
                error_A1 = vel_refA-velm_A
                error_B1 = vel_refB-velm_B
                control_A_input = (error_A1*Kp)+(Kd*error_A1/(dticks_A/1000000))+(Ki*error_A1*dticks_A/1000000)
                control_B_input = (error_B1*Kp)+(Kd*error_B1/(dticks_B/1000000))+(Ki*error_B1*dticks_B/1000000)
                duty_A_input = max(-100, min(100, control_A_input)) # ensure control loop duty is less than 100 absolute value
                duty_A_share.put(duty_A_input) # put input into duty A share
                duty_B_input = max(-100, min(100, control_B_input)) # ensure control loop duty is less than 100 absolute value
                duty_B_share.put(duty_B_input) # put input into duty B share
                buf+=1
                if buf >= 20:
                    duty_A_share.put(0)
                    duty_B_share.put(0)
                
        yield 0
        
# Task for Communication/Printing
def comms_task():
    while True:
        # Collect and print data from queues if available
        vel_A_queue.put(velm_A_share.get())
        vel_B_queue.put(velm_B_share.get())
        pos_A_queue.put(pos_A_share.get())
        pos_B_queue.put(pos_B_share.get())
        delta_A_queue.put(delta_A_share.get())
        delta_B_queue.put(delta_B_share.get())
        err_A_queue.put(err_A_share.get())
        err_B_queue.put(err_B_share.get())
        eula_x_queue.put(eula_x_share.get())
        d_queue.put(d_share.get())
        duty_A_queue.put(duty_A_share.get())
        duty_B_queue.put(duty_B_share.get())
        bump_queue.put(bump_share.get())
        theta_queue.put(theta_share.get())
        if pos_A_queue.any():
            pos_A = pos_A_queue.get()
            delta_A = delta_A_queue.get()
            vel_A = vel_A_queue.get()
            err_A = err_A_queue.get()
            duty_A = duty_A_queue.get()
            print(f"Motor A - Duty: {duty_A}, Delta: {delta_A}, Vel: {vel_A}, Err: {err_A}")
        
        if pos_B_queue.any():
            pos_B = pos_B_queue.get()
            delta_B = delta_B_queue.get()
            vel_B = vel_B_queue.get()
            err_B = err_B_queue.get()
            duty_B = duty_B_queue.get()
            print(f"Motor B - Duty: {duty_B}, Delta: {delta_B}, Vel: {vel_B}, Err: {err_B}")
            
        if eula_x_queue.any():  # Check if there is data in the share
            eula_x = eula_x_queue.get()
            print(f"Euler Angle - X: {eula_x}")
            theta = theta_queue.get()
            print(f"Encoder Theta: {theta}")
        yield 0

if __name__ == '__main__':
    print("Program Start... Waiting 1 sec") # startup message
    time.sleep(1) # allow for romi setup
    # setup global variables
    global perod
    perod = 10 # in ms used in task scheduler
    Kp = 3.92 # gain proportional
    Kd = .00001 # gain derivative
    Ki = 10  # gain integral
    # Set up timers and motor instances
    tim_1 = Timer(1, freq=20000)
    global mot_A
    mot_A = Motor_Driver.Motor(tim_1, Pin.cpu.A8, Pin.cpu.B10, Pin.cpu.C6)
    
    tim_2 = Timer(2, freq=20000)
    global mot_B
    mot_B = Motor_Driver.Motor(tim_2, Pin.cpu.A5, Pin.cpu.C2, Pin.cpu.C3)
    
    tim_3 = Timer(3, period=65535, prescaler=0)
    global enc_A
    enc_A = Encoder_Driver.Encoder(tim_3, Pin.cpu.B4, Pin.cpu.B5)
    
    tim_5 = Timer(5, period=65535, prescaler=0)
    global enc_B
    enc_B = Encoder_Driver.Encoder(tim_5, Pin.cpu.A0, Pin.cpu.A1)
    
    global bump_pins
    bump_pins = [
         Pin(Pin.cpu.A4, Pin.IN, Pin.PULL_UP),
         Pin(Pin.cpu.B0, Pin.IN, Pin.PULL_UP),
         Pin(Pin.cpu.C1, Pin.IN, Pin.PULL_UP),
         Pin(Pin.cpu.B3, Pin.IN, Pin.PULL_UP),
         Pin(Pin.cpu.C7, Pin.IN, Pin.PULL_UP),
         Pin(Pin.cpu.B6, Pin.IN, Pin.PULL_UP)
    ]
    #global bmp0
    #bmp0 = Pin(Pin.cpu.A4, mode=Pin.OUT_PP)
    
    global ls
    ls = linesensor_driver.LS()
    
    global imu
    imu = BNO055_Driver.IMU(1) # setup IMU
    imu.reset_IMU() # reset on startup
    time.sleep(.5)
    imu.set_mode(imu.NDOF_MODE) # set mode to nine degree of freedom mode
    imu.write_cal_co() # write pre-determined calibration coefficients
    
    # Shared variables and queues
    duty_A_share = task_share.Share('f', name="Motor_A_Duty")
    duty_B_share = task_share.Share('f', name="Motor_B_Duty")

    pos_A_share = task_share.Share('f', name="Motor_A_Pos")
    pos_B_share = task_share.Share('f', name="Motor_B_Pos")
    delta_A_share = task_share.Share('f', name="Motor_A_Delta")
    delta_B_share = task_share.Share('f', name="Motor_B_Delta")
    velm_A_share = task_share.Share('f', name="Motor_A_Vel_Meas")
    velm_B_share = task_share.Share('f', name="Motor_B_Vel_Meas")
    ticks_A_share = task_share.Share('f', name="Delta_Ticks_A")
    ticks_B_share = task_share.Share('f', name="Delta_Ticks_B")
    theta_share = task_share.Share('f', name="Enc_A_Theta")
    
    eula_x_share = task_share.Share('f', name="Eula_X")
    err_A_share = task_share.Share('f', name="Error_A")
    err_B_share = task_share.Share('f', name="Error_B")
    
    d_share = task_share.Share('f', name="Line_Sensor_D")
    bump_share = task_share.Share('i', name="Bump")
    bump_end_share = task_share.Share('i', name="Bump_End")
    end_share = task_share.Share('f', name="End_Check")
    
    pos_A_queue = cqueue.FloatQueue(500, name="Pos_A_Queue")
    delta_A_queue = cqueue.FloatQueue(500, name="Delta_A_Queue")
    vel_A_queue = cqueue.FloatQueue(500, name="Vel_A_Queue")
    duty_A_queue = cqueue.FloatQueue(500, name="Duty_A_Queue")
    
    pos_B_queue = cqueue.FloatQueue(500, name="Pos_B_Queue")
    delta_B_queue = cqueue.FloatQueue(500, name="Delta_B_Queue")
    vel_B_queue = cqueue.FloatQueue(500, name="Vel_B_Queue")
    duty_B_queue = cqueue.FloatQueue(500, name="Duty_B_Queue")
    
    theta_queue = cqueue.FloatQueue(500, name="Theta_Queue")
    
    eula_x_queue = cqueue.FloatQueue(500, name="Eula_X_Queue")
    eula_y_queue = cqueue.FloatQueue(500, name="Eula_Y_Queue")
    eula_z_queue = cqueue.FloatQueue(500, name="Eula_Z_Queue")
    
    gyro_x_queue = cqueue.FloatQueue(500, name="Gyro_X_Queue")
    gyro_y_queue = cqueue.FloatQueue(500, name="Gyro_Y_Queue")
    gyro_z_queue = cqueue.FloatQueue(500, name="Gyro_Z_Queue")
    
    err_A_queue = cqueue.FloatQueue(500, name="Error_A_Queue")
    err_B_queue = cqueue.FloatQueue(500, name="Error_B_Queue")
    
    d_queue = cqueue.FloatQueue(500, name="Line_Sensor_D_Queue")
    bump_queue = cqueue.FloatQueue(500, name="Bump_Queue")
    
    # Set up tasks
    mot_enc_A = cotask.Task(lambda: mot_enc_task_A(), name='Motor_Encoder_A', priority=1, period=perod)
    mot_enc_B = cotask.Task(lambda: mot_enc_task_B(), name='Motor_Encoder_B', priority=1, period=perod)
    control = cotask.Task(lambda: control_task(15), name='Control_Loop', priority=1, period=perod) #15 before
    imu_task = cotask.Task(lambda: read_imu_task(), name="IMU Task", priority=1, period=perod)
    # abso_task = cotask.Task(lambda: abso_position(), name="Abso_Pos_Task", priority=1, period=perod)
    ls_task = cotask.Task(lambda: read_ls_task(), name="LS_Task", priority=1, period=perod)
    bp_task = cotask.Task(lambda: bump_task(), name="Bump_Task", priority=1, period=perod)
    
    comms_t = cotask.Task(lambda: comms_task(), name='Comms_Task', priority=2, period=1000)
    
    # Add tasks to the scheduler
    cotask.task_list.append(mot_enc_A)
    cotask.task_list.append(mot_enc_B)
    cotask.task_list.append(control)
    # cotask.task_list.append(abso_task)
    cotask.task_list.append(imu_task)
    cotask.task_list.append(ls_task)
    cotask.task_list.append(bp_task)
    cotask.task_list.append(comms_t)
    
    # Run the memory garbage collector to ensure memory is as defragmented as
    # possible before the real-time scheduler is started
    gc.collect()

    # Run the scheduler with the chosen scheduling algorithm. Quit if ^C pressed
    while True:
        try:
            cotask.task_list.pri_sched()
        except KeyboardInterrupt:
            break

