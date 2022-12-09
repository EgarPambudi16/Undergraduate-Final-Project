import cv2
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.lines import Line2D
import copy

def amg_process(pixels, temp_threshold):
    norm_pix = [float(pixels[x]) for x in range(0, 64)]

    ## Display AMG8833 Image
    max_temp = np.max(norm_pix)
    pixels_data = np.reshape(norm_pix, (8,8))
    frame = np.array(pixels_data * (255/80), dtype=np.uint8)
    frame = cv2.resize(frame, (800, 800))

    ## Threshold Temperature
    thres_val = temp_threshold*(256/80) - 1
    ret, thres = cv2.threshold(frame, thres_val, 255, cv2.THRESH_BINARY)
    thres_copy = copy.deepcopy(thres)
    thres_bgr = cv2.cvtColor(thres_copy, cv2.COLOR_GRAY2BGR)
    contours, hierarchy = cv2.findContours(thres_copy, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #Detect Contours
    return frame, thres_bgr, contours, max_temp

def threshold_box(frame, res, bounding_box, center):
    x, y, w, h = bounding_box
    x_center, y_center = center
    cv2.drawContours(frame, [res], 0, (255, 255, 255), 2) #Draw Contours 
    cv2.rectangle(frame, (x, y),(x+w, y+h), (255, 0, 0), 4) #Draw Rectangle
    cv2.circle(frame, (x_center, y_center), 3, (255, 0, 0)) #Draw Center Dot
    cv2.line(frame, (400, 400), (x_center, y_center), (0, 0, 255), 2) #Draw Distance Line
    return frame

def threshold_gui(frame, temp, temp_threshold, motion_status, position_estimation):
    if temp >= temp_threshold:
        temp_str = str(temp)
    else:
        temp_str = "None"

    if motion_status == 1:
        status_str = "Crazyflie Running!"
    elif motion_status == 2:
        status_str = "Crazyflie Tumbling!"
    elif motion_status == 3:
        status_str = "Crazyflie Centering!"
    elif motion_status == 4:
        status_str = "Crazyflie Chasing!"

    temp_text = "Measured Temperature: " + temp_str
    status_text = "Status: " + status_str
    # position_text = "Position: " + str(position_estimation[0]) + ", " + str(position_estimation[1]) + ", " + str(position_estimation[2])
    cv2.putText(frame, temp_text, (500, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv2.putText(frame, status_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv2.putText(frame, "Position: ", (10, 780), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(frame, str(position_estimation[0]), (100, 780), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(frame, str(position_estimation[1]), (300, 780), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(frame, str(position_estimation[2]), (500, 780), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return frame

def flight_data_plot(epoch):
    dataFrame = pd.read_csv("../Undergraduate-Final-Project/Code/Log/log" + str(epoch) + "1.csv", header=None)
    legend_element = [Line2D([0], [0], color='r', linewidth=3, label='Start'),
                    Line2D([0], [0], color='g', linewidth=3, label='Finish'),
                    Line2D([0], [0], color='y', marker='o', label='Run'),
                    Line2D([0], [0], color='m', marker='o', label='Tumble'),
                    Line2D([0], [0], color='k', marker='o', label='Centering'),
                    Line2D([0], [0], color='b', marker='o', label='Chasing'),]
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    title_str = "Flight Log " + str(epoch)
    for i in range(0,len(dataFrame)):
        x = dataFrame[0][i]
        y = dataFrame[1][i]
        z = dataFrame[2][i]
        motion_status = dataFrame[3][i]
        if i == 0:
            ax.scatter(x, y, z, s=100, c='r', marker='x')
        elif i == (len(dataFrame)-1):
            ax.scatter(x, y, z, s=100, c='g', marker='s')
        elif motion_status == 1:
            ax.scatter(x, y, z, s=30, c='y')
        elif motion_status == 2:
            ax.scatter(x, y, z, s=30, c='m')
        elif motion_status == 3:
            ax.scatter(x, y, z, s=30, c='k')
        elif motion_status == 4:
            ax.scatter(x, y, z, s=30, c='b')

    ax.set_xlim3d(-1.2, 1.2)
    ax.set_ylim3d(-1.2, 1.2)
    ax.set_zlim3d(0, 1.5)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")
    ax.set_title(title_str)
    ax.legend(handles=legend_element)
    plt.savefig('../Undergraduate-Final-Project/Code/Plot' + str(epoch) + 'flightplot.png')
    plt.show()

if __name__ == '__main__':
    flight_data_plot(1)
