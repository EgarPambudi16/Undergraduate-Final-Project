import numpy as np
import socket
import matplotlib.pyplot as plt
import time

def amg_process(byte_data):
    temp_str = str(byte_data, 'utf8')
    temp_vector = temp_str.split(";")[:-1]
    temp_vector = [float(x) for x in temp_vector]
    return temp_vector

def amg_detection(pix_data, tres):
    t_pix_data = pix_data.transpose()

    back_ave = np.average(pix_data)
    upper_ave = np.average(pix_data[0:3])
    lower_ave = np.average(pix_data[5:8])
    left_ave = np.average(t_pix_data[0:3])
    right_ave = np.average(t_pix_data[5:8])
    center_ave = np.average([x[2:6] for x in pix_data[2:6]])

    print(back_ave, upper_ave, lower_ave, left_ave, right_ave, center_ave)

    if center_ave > (back_ave+tres):
        state = 1
    elif upper_ave > (back_ave+tres):
        state = 2
    elif lower_ave > (back_ave+tres):
        state = 3
    elif left_ave > (back_ave+tres):
        state = 4
    elif right_ave > (back_ave+tres):
        state = 5
    else:
        state = 0

    return state

norm_pix = []
cal_vec = []
cal_pix = []
kk = 0
cal_size = 10

HOST = '0.0.0.0' 
PORT = 8080      
s = socket.socket()
s.bind((HOST, PORT))
s.listen(0)

plt.ion()

while True:
    client, addr = s.accept()
    content = client.recv(512)
    norm_pix = amg_process(content)
    if kk==0: #Callibration Algoritm
        print("Sensor should have clear path to calibrate against environment")
        figure, graph = plt.subplots(figsize=(10, 8))
        graph = plt.imshow(np.reshape(np.repeat(0, 64), (8, 8)),cmap=plt.cm.cool)
        plt.colorbar()
        plt.clim(1, 8)
        plt.draw()
    # print(norm_pix)
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
    #print(pixels_data)
    state = amg_detection(pixels_data, tres=0.5)
    print(state)
    graph.set_data(pixels_data)
    figure.canvas.draw()
    figure.canvas.flush_events()
    cal_pix = []
    time.sleep(0.1)

s.close()