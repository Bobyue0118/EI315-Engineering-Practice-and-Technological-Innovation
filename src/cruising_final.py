# -*- coding: utf-8 -*-
import cv2
import time
import numpy as np
from driver import driver


# --- init --- #
cap = cv2.VideoCapture(1)
d = driver()
ROW = 465
IMAGE_CENTER = 320
count = 100
COL_NUM = 640
ROW_NUM = 480
INF = 1e7
# THRESHOLD = 20

P = 0.004
D = 0
error_ = 0
road_length = 60
lam = 0.7
last_direction = None

import os
import stat

filePath = './image'
for fileList in os.walk(filePath):
   for name in fileList[2]:
    os.chmod(os.path.join(fileList[0],name), stat.S_IWRITE)
    os.remove(os.path.join(fileList[0],name))

# -- function -- #
def threshold(value, MIN=-1, MAX=1):
    """
    threshold function
    :param value:
    :param MIN:
    :param MAX:
    :return:
    """
    if value < MIN:
        return MIN
    if value > MAX:
        return MAX
    return value

def find_block(color, row):#寻找图片中row行处连续黑色的块
    index_start_array = []
    index_end_array = []
    lengeth_array = []
    start = 0
    end = 0
    flag = False
    for i in range(COL_NUM):
        if(color[i]==0):
            if(flag==False):
                flag = True
                start = i
        else:
            if(flag ==True):
                flag = False
                end = i-1                
                index_start_array.append(start)
                index_end_array.append(end)
                lengeth_array.append(end-start+1)
    if(flag==True):
        index_start_array.append(start)
        index_end_array.append(COL_NUM-1)
        lengeth_array.append(COL_NUM-1-start+1)
    return index_start_array, index_end_array, lengeth_array #index_start_array, index_end_array，length_array分别为黑色块的开始、结束的坐标和长度

def calError(image, row):
    """
    calculate the error for P(I)D control
    :param image: dst
    :param row: ROW
    :return: error
    """
    global road_length, last_direction
    color = image[row]
    center = 320

    white_count = np.sum(color == 0)
    white_index = np.where(color == 0)
    
    # index_start_array = []
    # index_end_array = []
    # lengeth_array = []
    # start = 0
    # end = 0
    # flag = False
    # for i in range(COL_NUM):
    #     if(color[i]==0):
    #         if(flag==False):
    #             flag = True
    #             start = i
    #     else:
    #         if(flag ==True):
    #             flag = False
    #             end = i-1                
    #             index_start_array.append(start)
    #             index_end_array.append(end)
    #             lengeth_array.append(end-start+1)
    # if(flag==True):
    #     index_start_array.append(start)
    #     index_end_array.append(COL_NUM-1)
        # lengeth_array.append(COL_NUM-1-start+1)
    
    index_start_array, index_end_array, lengeth_array = find_block(color, row)
    
    if len(lengeth_array)==0: #没有找到黑色块
        if last_direction:
            direction = last_direction #设置方向为0
        else:
            direction = 0
        center = 320
        print('ERROR: white_count == 0!')
    elif len(lengeth_array)==1: #只找到一个
        row_below = row+5
        color_below = image[row_below]
        index_start_array_below, index_end_array_below, lengeth_array_below = find_block(color_below, row_below)
        mediam = (index_end_array[0] + index_start_array[0]) /2
        
        near_index = -1
        near_dis = INF
        for i in range(len(index_start_array_below)):
            mediam_below = (index_end_array_below[i] + index_start_array_below[i])/2
            if abs(mediam_below-mediam) < near_dis:
                near_dis = abs(mediam_below - mediam)
                near_index = i
        if near_index == -1:
            center = mediam
        else:
            mediam_below = (index_end_array_below[near_index] + index_start_array_below[near_index])/2
            if mediam > mediam_below:
                left_mediam = mediam
                right_mediam = left_mediam + road_length
                right_mediam = max(COL_NUM-1, left_mediam)
            else:
                right_mediam = mediam
                left_mediam = right_mediam - road_length
                left_mediam = min(left_mediam, 0)
            center = (left_mediam + right_mediam)/2
            
        center = max(0, center)
        center = min(center, COL_NUM-1)
        direction = center - IMAGE_CENTER
    else:# 找到两个及以上
        first = 0
        second = 0
        first_index = None
        second_index = None
        for index, length in enumerate(lengeth_array):
            if(length>=first):
                second = first
                first = length
                second_index = first_index
                first_index = index
            elif(length>second):
                second = length
                second_index = index
        if(first_index>second_index):
            tmp = first_index
            first_index = second_index 
            second_index = tmp
        left_mediam = (index_start_array[first_index] + index_end_array[first_index])/2
        right_mediam = (index_start_array[second_index] + index_end_array[second_index])/2
        if road_length == None:
            road_length = right_mediam - left_mediam
        else:
            road_length = lam*road_length + (1-lam)*(right_mediam-left_mediam)
        center = (left_mediam + right_mediam)/2
        direction = center - IMAGE_CENTER
    # # 防止white_count = 0的报错
    # if white_count != 0:
    #     center = (white_index[0][white_count - 1] + white_index[0][0]) / 2
    #     direction = center - IMAGE_CENTER
    #     cv2.drawMarker(frame, (center, row), (0, 255, 0), markerType=1, thickness=2)
    # else:
    #     direction = 0
    #     print('ERROR: white_count == 0!')
    last_direction = direction
    return direction, center

counter = 0
# --- main --- #
try:
    while True:
        counter += 1
        if counter < 10:
            _, frame = cap.read()
            d.setStatus(motor=0.0, servo=0, mode="speed")
            time.sleep(0.4)
        else:
            # -- image process -- #
            _, frame = cap.read()
            frame = cv2.flip(frame, -1)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            cv2.imwrite('./image_raw/{}.jpg'.format(count), gray)
            kernel = np.ones((8, 8), np.uint8)
            retval, dst = cv2.threshold(gray, 130, 255, cv2.THRESH_BINARY)
            #dst = cv2.morphologyEx(dst, cv2.MORPH_CLOSE, kernel)
            #dst = cv2.morphologyEx(dst, cv2.MORPH_OPEN, kernel)
            #dst = cv2.blur(dst,(8, 8))
            dst = cv2.erode(dst, None, iterations=6)
            dst = cv2.dilate(dst, None, iterations=2)
            

            # -- image show command -- #
            # dst: binary image; frame: RGB image with mark
            #cv2.imshow('dst', frame)
            #cv2.waitKey(3)

            # -- control -- #
            # -- PD -- #
            error, cen = calError(dst, ROW)
            servo_now = - P * error - D * (error - error_)
            servo_now = threshold(servo_now)
            error_ = error

            # --- sleep --- #
            """high_speed = 0.1
            low_speed = 0.05
            high_servo = 0.4
            low_servo = 0.05
            if abs(servo_now) < low_servo:
                speed = high_speed
            elif abs(servo_now) > high_servo:
                speed = low_speed
            else:
                speed = low_speed + (high_speed - low_speed)/(high_servo - low_servo)*(servo_now - low_servo)"""
            d.setStatus(motor=0.05, servo=servo_now, mode="speed")
            time.sleep(0.4)
            
            cv2.drawMarker(dst, (int(cen), ROW), (0, 255, 0), markerType=1, thickness=3)
            
            name = str(count)
            name += '_'+str(servo_now)
            cv2.imwrite('./image/{}.jpg'.format(count), dst)
            
            count += 1

            # --- print --- #
            print("---{} error: {} servo: {} ---".format(name, error, servo_now))

except KeyboardInterrupt as e:
    print(repr(e))

# --- release --- #
cap.release()
print('--- In main.py: cap released successfully ---')
cv2.destroyAllWindows()
print('--- In main.py: windows destroyed successfully ---')
d.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")
print('--- In main.py: status set successfully ---')
d.close()
print('--- In main.py: driver closed successfully ---')
del d
print('--- In main.py: over ---')
