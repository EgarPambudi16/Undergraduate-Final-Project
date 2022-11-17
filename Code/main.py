import numpy as np
import socket
import cv2
import time
import copy

def amg_process(byte_data):
    temp_str = str(byte_data, 'utf8')
    temp_vector = temp_str.split(";")[:-1]
    temp_vector = [float(x) for x in temp_vector]
    return temp_vector

# Initialize Pixels and Callibration Algorithm
norm_pix = []
cal_vec = []
cal_pix = []
kk = 0
cal_size = 10

frame = np.zeros((8,8))

HOST = '0.0.0.0' 
PORT = 8080      
s = socket.socket()
s.bind((HOST, PORT))
s.listen(0)

while True:
    a = time.time()
    client, addr = s.accept()
    content = client.recv(512)
    norm_pix = amg_process(content)
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

    #Display Graph
    pixels_data = np.reshape(cal_pix, (8,8))
    frame = np.array(pixels_data * (255/8), dtype=np.uint8)
    cal_pix = []
    print(frame)
    frame = cv2.resize(frame, (800, 800))
    ret, thres = cv2.threshold(frame, 100, 255, cv2.THRESH_BINARY)
    
    thres_copy = copy.deepcopy(thres)
    thres_bgr = cv2.cvtColor(thres_copy, cv2.COLOR_GRAY2BGR)
    contours, hierarchy = cv2.findContours(thres_copy, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #detecting contours
    length = len(contours)
    maxArea = -1
    if length > 0:
        for i in range(length):  # find the biggest contour (according to area)
            con_temp = contours[i]
            area = cv2.contourArea(con_temp)
            if area > maxArea:
                maxArea = area
                ci = i

        res = contours[ci]
        x, y, w, h = cv2.boundingRect(res)
        cv2.drawContours(thres_bgr, [res], 0, (255, 255, 255), 2) #Draw Contours 
        cv2.rectangle(thres_bgr, (x, y),(x+w, y+h), (255, 0, 0), 4) #Draw Rectangle

    b = time.time()
    fps = str(int(1/(b-a)))
    cv2.putText(thres_bgr, fps, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 1)
    cv2.imshow("Threshold", thres_bgr)
    cv2.imshow("Frame", frame)
    
    if cv2.waitKey(1) == 27:
        break

s.close()