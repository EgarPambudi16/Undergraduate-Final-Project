import logging
import time
import random
import sys
import numpy as np
import cv2
import copy
from threading import Event
from simple_pid import PID

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

## Initialize Crazyflie
###########################################################
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
deck_attached_event = Event()
logging.basicConfig(level=logging.ERROR)

###########################################################
DEFAULT_HEIGHT = 0.8
XY_VEL = 0.2
take_off_status = False
heat_status = False
motion_status = 1
position_estimate = [0, 0, 0]
x_dis_arr = np.zeros(10)
y_dis_arr = np.zeros(10)
xi = 0
yi = 0
pixels = np.zeros(64)
norm_pix = []
cal_vec = []
cal_pix = []
kk = 0
cal_size = 10

## Control and Motion Function
###########################################################
def pid_controller(x_distance, y_distance):
    global xi, yi
    xi = (xi+1) % 10
    yi = (yi+1) % 10
    x_dis_arr[xi] = x_distance
    y_dis_arr[yi] = y_distance
    x_ave = np.average(x_dis_arr)
    y_ave = np.average(y_dis_arr)

    pid_z_vel = PID(Kp=0.0002, Ki=0, Kd=0, setpoint=0, sample_time=0.05)
    pid_yaw_rate = PID(Kp=0.02, Ki=0, Kd=0, setpoint=0, sample_time=0.05)
    z_vel = pid_z_vel(-y_ave)
    yaw_rate = pid_yaw_rate(-x_ave)

    return z_vel, yaw_rate

# def take_off_sequence(scf):
#     mc.take_off(height=DEFAULT_HEIGHT, velocity=1)
#     print("Crazyflie Take-Off!")
#     time.sleep(2)

# def landing_sequence(scf):
#     with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
#         mc.land(0.4)
#         print("Crazyflie Landing!")
#         time.sleep(2)

def motion_run(mc, z_vel, yaw_rate):
    global motion_status
    motion_status = 1
    mc.start_linear_motion(velocity_x_m=0, velocity_y_m=0, velocity_z_m=z_vel, rate_yaw=yaw_rate)
    print("Crazyflie Running!")

def motion_tumble(mc):
    global motion_status
    global heat_status
    motion_status = 2
    tt = 0
    r = random.random()
    while not heat_status:
        [t1, t2] = [0, 0]
        t1 = time.time()
        if r < 0.5:
            mc.start_turn_right(30)
            c+=1
        else:
            mc.start_turn_left(30)
            c+=1
        print("Crazyflie Tumbling!")
        t2 = time.time()
        tt += (t2 - t1)
        if heat_status or tt <= 12:
            break
    mc.stop()
    time.sleep(1) 

def motion_centering(mc, z_vel, yaw_rate):
    global motion_status
    global heat_status
    if not heat_status:
        return
    else:
        motion_status = 3
        print("Crazyflie Centering!")
        mc.start_linear_motion(0, 0, z_vel, yaw_rate)

## Logging Function
###########################################################
def log_pos_callback(timestamp, data, logconf):
    print(data)
    global position_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']
    position_estimate[2] = data['stateEstimate.z']

def log_amg_callback1(timestamp, data, logconf):
    #print(data)
    global pixels
    for x in range (0, 16):
        pixels[63-x] = data['amgdeck.new_pixels[' + str(x) + ']']

def log_amg_callback2(timestamp, data, logconf):
    #print(data)
    global pixels
    for x in range (16, 32):
        pixels[63-x] = data['amgdeck.new_pixels[' + str(x) + ']']

def log_amg_callback3(timestamp, data, logconf):
    #print(data)
    global pixels
    for x in range (32, 48):
        pixels[63-x] = data['amgdeck.new_pixels[' + str(x) + ']']

def log_amg_callback4(timestamp, data, logconf):
    #print(data)
    global pixels
    for x in range (48, 64):
        pixels[63-x] = data['amgdeck.new_pixels[' + str(x) + ']']

def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')

def param_deck_amg(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('AMG is attached!')
    else:
        print('AMG is NOT attached!')

def logconf_amg_init(period):
    logconf_amg1 = LogConfig(name='Pixels', period_in_ms=period)
    for x in range (0, 16):
        amg_variable = 'amgdeck.new_pixels[' + str(x) + ']'
        logconf_amg1.add_variable(amg_variable, 'uint8_t')
    scf.cf.log.add_config(logconf_amg1)
    logconf_amg1.data_received_cb.add_callback(log_amg_callback1)

    logconf_amg2 = LogConfig(name='Pixels', period_in_ms=period)
    for x in range (16,32):
        amg_variable = 'amgdeck.new_pixels[' + str(x) + ']'
        logconf_amg2.add_variable(amg_variable, 'uint8_t')
    scf.cf.log.add_config(logconf_amg2)
    logconf_amg2.data_received_cb.add_callback(log_amg_callback2)

    logconf_amg3 = LogConfig(name='Pixels', period_in_ms=period)
    for x in range (32,48):
        amg_variable = 'amgdeck.new_pixels[' + str(x) + ']'
        logconf_amg3.add_variable(amg_variable, 'uint8_t')
    scf.cf.log.add_config(logconf_amg3)
    logconf_amg3.data_received_cb.add_callback(log_amg_callback3)

    logconf_amg4 = LogConfig(name='Pixels', period_in_ms=period)
    for x in range (48,64):
        amg_variable = 'amgdeck.new_pixels[' + str(x) + ']'
        logconf_amg4.add_variable(amg_variable, 'uint8_t')
    scf.cf.log.add_config(logconf_amg4)
    logconf_amg4.data_received_cb.add_callback(log_amg_callback4)

    return logconf_amg1, logconf_amg2, logconf_amg3, logconf_amg4    

if __name__ == '__main__':

    ## Initialize Crazyflie Connection and Parameter
    ###########################################################
    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                         cb=param_deck_flow) 
        scf.cf.param.add_update_callback(group='deck', name='amgdeck',
                                         cb=param_deck_amg)
        time.sleep(1)

        logconf_pos = LogConfig(name='Position', period_in_ms=20)
        logconf_pos.add_variable('stateEstimate.x', 'float')
        logconf_pos.add_variable('stateEstimate.y', 'float')
        logconf_pos.add_variable('stateEstimate.z', 'float')
        scf.cf.log.add_config(logconf_pos)
        logconf_pos.data_received_cb.add_callback(log_pos_callback)

        logconf_amg1, logconf_amg2, logconf_amg3, logconf_amg4 = logconf_amg_init(100)

        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        logconf_pos.start()
        logconf_amg1.start()
        logconf_amg2.start()
        logconf_amg3.start()
        logconf_amg4.start()
        
        # with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        while True:
            a = time.time()
            norm_pix = [float(pixels[x]) for x in range(0, 64)]
            if kk==0: #Calibration Algoritm
                print("Clear the Sensor Path for Background Calibration")
            if kk<cal_size+1:
                kk+=1
            if kk==1:
                cal_vec = norm_pix    
                continue                    
            elif kk<=cal_size:
                for xx in range(0,len(norm_pix)):
                    cal_vec[xx]+=norm_pix[xx]
                    if kk==cal_size:
                        cal_vec[xx] = cal_vec[xx]/cal_size
                continue
            else:
                [cal_pix.append(norm_pix[x]-cal_vec[x]) for x in range(0,len(norm_pix))]
                if min(cal_pix)<0:
                    for y in range(0,len(cal_pix)):
                        cal_pix[y]+=abs(min(cal_pix))

            ## Display AMG8833 Image
            pixels_data = np.reshape(cal_pix, (8,8))
            #print(pixels_data)
            frame = np.array(pixels_data * (255/80), dtype=np.uint8)
            cal_pix = []
            #print(frame)
            frame = cv2.resize(frame, (800, 800))

            ## Threshold Temperature
            ret, thres = cv2.threshold(frame, 96, 255, cv2.THRESH_BINARY) #Threshold to 30 degrees
            thres_copy = copy.deepcopy(thres)
            thres_bgr = cv2.cvtColor(thres_copy, cv2.COLOR_GRAY2BGR)
            contours, hierarchy = cv2.findContours(thres_copy, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #Detect Contours
            length = len(contours)
            maxArea = -1
            if length > 0:
                for i in range(length):  #Finding the Biggest Contour
                    con_temp = contours[i]
                    area = cv2.contourArea(con_temp)
                    if area > maxArea:
                        maxArea = area
                        ci = i

                res = contours[ci]
                x, y, w, h = cv2.boundingRect(res)
                x_center = int(x+w/2)
                y_center = int(y+h/2)
                x_amg_distance = int(x_center - 400)
                y_amg_distance = int(400 - y_center)

                cv2.drawContours(thres_bgr, [res], 0, (255, 255, 255), 2) #Draw Contours 
                cv2.rectangle(thres_bgr, (x, y),(x+w, y+h), (255, 0, 0), 4) #Draw Rectangle
                cv2.circle(thres_bgr, (x_center, y_center), 3, (255, 0, 0)) #Draw Center Dot
                cv2.line(thres_bgr, (400, 400), (x_center, y_center), (0, 0, 255), 2) #Draw Distance Line

                heat_status = True
                z_vel, yaw_rate = pid_controller(x_amg_distance, y_amg_distance)
                # motion_run(mc, z_vel, yaw_rate)
                # print("X Distance: ", x_amg_distance)
                # print("Y Distance: ", y_amg_distance)
                # print("Z Velocity: ", z_vel)
                # print("Yaw Rate: ", yaw_rate)
            else:
                heat_status = False
                # mc.stop()
                
            b = time.time()
            fps = 1 / (b - a)
            cv2.putText(thres_bgr, str(fps), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 1)
            #cv2.putText(thres_bgr, str(max_temp), (box[0], box[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 1)
            cv2.imshow("Threshold", thres_bgr)
            cv2.imshow("Frame", frame)
            if cv2.waitKey(1) == 27:  
                break
            # mc.land(0.5)

        logconf_pos.stop()
        logconf_amg1.stop()
        logconf_amg2.stop()
        logconf_amg3.stop()
        logconf_amg4.stop()
        
        sys.exit(1)