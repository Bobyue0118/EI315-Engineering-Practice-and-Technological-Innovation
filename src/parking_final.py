# -*- coding: utf-8 -*-
import cv2
import time
import numpy as np
from driver import driver

import argparse


# --- init --- #
cap = cv2.VideoCapture(0)
d = driver()
im_saving_count = 100
parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('integers', metavar='N', type=int, nargs='+',
                    help='an integer for the accumulator')

args = parser.parse_args()

# --- main --- #
try:
    if args.integers[0] == 3:  # 车位3
        for i in range(3):
            print("--- 倒车转向中 ---")
            d.setStatus(motor=-0.1, servo=-0.5, mode="speed")
            time.sleep(0.8)

        for i in range(3):
            print("--- 回正 ---")
            d.setStatus(motor=-0.08, servo=0.45, mode="speed")
            time.sleep(0.8)

        time.sleep(0.8)
    elif args.integers[0] == 2:  # 车位2
        for i in range(3):
            print("--- 倒车转向中 ---")
            d.setStatus(motor=-0.1, servo=0.5, mode="speed")
            time.sleep(0.8)

        for i in range(3):
            print("--- 回正 ---")
            d.setStatus(motor=-0.08, servo=-0.45, mode="speed")
            time.sleep(0.8)

        time.sleep(0.8)

    elif args.integers[0] == 1:  # 车位1
        for i in range(4):
            print("--- 倒车转向中 ---")
            d.setStatus(motor=-0.1, servo=1, mode="speed")
            time.sleep(0.8)

        for i in range(4):
            print("--- 回正 ---")
            d.setStatus(motor=-0.08, servo=-0.95, mode="speed")
            time.sleep(0.8)

        time.sleep(0.8)
    elif args.integers[0] == 4:  # 车位4
        for i in range(4):
            print("--- 倒车转向中 ---")
            d.setStatus(motor=-0.1, servo=-1, mode="speed")
            time.sleep(0.8)

        for i in range(4):
            print("--- 回正 ---")
            d.setStatus(motor=-0.08, servo=0.95, mode="speed")
            time.sleep(0.8)

        time.sleep(0.6)

    d.setStatus(motor=0, servo=0, mode="speed") 
    time.sleep(2)
    print("--- stop for a while ---")

    # 此时应该正对车位
    while True:
        # -- image process -- #
        _, frame = cap.read()
        # frame = cv2.flip(frame, -1)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        retval, dst = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
        dst = cv2.erode(dst, None, iterations=2)
        dst = cv2.dilate(dst, None, iterations=5)

        # 判断停车
        color = dst[265]
        white_count = np.sum(color == 255)
        black_count = np.sum(color == 0)
        
        print("black_count: {}".format(black_count))
        print("white_counte_count: {} {}".format(white_count, im_saving_count))
        
        cv2.imwrite("Parking/{}.jpg".format(im_saving_count), dst)

        threshold_num = 200
        if white_count < threshold_num and im_saving_count > 104:
            print("--- 停车 ---")
            d.setStatus(motor=0, servo=0, mode="speed")
            break

        im_saving_count += 1

        d.setStatus(motor=-0.03, servo=0, mode="speed")
        time.sleep(0.8)

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
