#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2021 Bitcraze AB
#
#  AI-deck demo
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License along with
#  this program; if not, write to the Free Software Foundation, Inc., 51
#  Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
#
#  Demo for showing streamed JPEG images from the AI-deck example.
#
#  By default this demo connects to the IP of the AI-deck example when in
#  Access point mode.
#
#  The demo works by opening a socket to the AI-deck, downloads a stream of
#  JPEG images and looks for start/end-of-color_img for the streamed JPEG images.
#  Once an image has been fully downloaded it's rendered in the UI.
#
#  Note that the demo firmware is continously streaming JPEG files so a single
#  JPEG image is taken from the stream using the JPEG start-of-color_img (0xFF 0xD8)
#  and the end-of-color_img (0xFF 0xD9).

import argparse
import time
import socket,os,struct, time
import numpy as np
import os

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils.power_switch import PowerSwitch
from threading import Event, Thread
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

from cflib.positioning.motion_commander import MotionCommander

import logging



uri = 'radio://0/80/2M/E7E7E7E701'


# Args for setting IP/port of AI-deck. Default settings are for when
# AI-deck is in AP mode.
parser = argparse.ArgumentParser(description='Connect to AI-deck JPEG streamer example')
parser.add_argument("-n",  default="192.168.4.1", metavar="ip", help="AI-deck IP")
parser.add_argument("-p", type=int, default='5000', metavar="port", help="AI-deck port")
parser.add_argument('--save', action='store_true', help="Save streamed images")
args = parser.parse_args()

deck_port = args.p
deck_ip = args.n

print("Connecting to socket on {}:{}...".format(deck_ip, deck_port))
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((deck_ip, deck_port))
print("Socket connected")

imgdata = None
data_buffer = bytearray()

def rx_bytes(size):
  data = bytearray()
  while len(data) < size:
    data.extend(client_socket.recv(size-len(data)))
  return data

import cv2

start = time.time()
count = 0
imgcount = 0
# Define chessboard size
square_size = 1.0
chessboard_size = (9, 6)
cube_size = square_size  # Make cube the size of a single square


camera_matrix = np.array([[190.191, 0, 160.525], [0, 187.639, 147.501], [0, 0, 1]])
dist_coeffs = np.array([[-0.08368914,  0.03974283, -0.0016545, 0.0050885,  -0.05173472]])
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size  # Scale if real square size is known

w, h = 344, 244
# Precompute undistortion and rectification maps
new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
map1, map2 = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, new_camera_matrix, (w, h), cv2.CV_16SC2)

real_diameter = 1.575  # cm
fx = camera_matrix[0, 0]
fy = camera_matrix[1, 1]
cx = camera_matrix[0, 2]
cy = camera_matrix[1, 2]

# Create directory to save images
save_path = "calibration_images"
os.makedirs(save_path, exist_ok=True)

deck_attached_event = Event()

def param_deck_flow(_, value_str):
    # Check whether positioning deck is connected or not

    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')

cflib.crtp.init_drivers()
    

lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
lg_stab.add_variable('stabilizer.roll', 'float')
lg_stab.add_variable('stabilizer.pitch', 'float')
lg_stab.add_variable('stabilizer.yaw', 'float')
lg_stab.add_variable('pm.vbat', 'float')
lg_stab.add_variable('pm.batteryLevel', 'uint8_t')

group = 'stabilizer'
name = 'estimator'

    
with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        
    scf.cf.param.add_update_callback(group="deck", name="bcLighthouse4",
                                cb=param_deck_flow)
        # main_thread = Thread(target=infinite_control, args=[scf, lg_stab])
        # main_thread.start()
    time.sleep(1)
        # base_commander(scf, lg_stab)


while(1):
    # First get the info
    packetInfoRaw = rx_bytes(4)
    [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)
    imgHeader = rx_bytes(length - 2)
    [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', imgHeader)

    if magic == 0xBC:
      imgStream = bytearray()

      while len(imgStream) < size:
          packetInfoRaw = rx_bytes(4)
          [length, dst, src] = struct.unpack('<HBB', packetInfoRaw)
          #print("Chunk size is {} ({:02X}->{:02X})".format(length, src, dst))
          chunk = rx_bytes(length - 2)
          imgStream.extend(chunk)
     
      count = count + 1
      meanTimePerImage = (time.time()-start) / count
      print(f"{count/(time.time()-start)} frames per second")

      if format == 0:
          gray = np.frombuffer(imgStream, dtype=np.uint8)   
          gray.shape = (244, 324)
          undistorted_gray = cv2.remap(gray, map1, map2, interpolation=cv2.INTER_LINEAR)
          undistorted_frame = cv2.cvtColor(undistorted_gray, cv2.COLOR_BayerBG2BGR)
          hsv_frame = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2HSV)

          lower_color = np.array([0, 0, 199])
          upper_color = np.array([179, 8, 255])
          mask = cv2.inRange(hsv_frame, lower_color, upper_color)

          segmented_frame = cv2.bitwise_and(undistorted_frame, undistorted_frame, mask=mask)

          gray = cv2.cvtColor(segmented_frame, cv2.COLOR_BGR2GRAY)
          _, thresh = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
          contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

          if contours:
              largest_contour = max(contours, key=cv2.contourArea)
              (center, radius) = cv2.minEnclosingCircle(largest_contour)
              center = (int(center[0]), int(center[1]))
              radius = int(radius)

        # 3D position estimate
              px_diameter = 2 * radius
              z = (fx * real_diameter) / px_diameter
              x = (center[0] - cx) * z / fx
              y = (center[1] - cy) * z / fy
              target = np.asarray([x, y, z])
              kp, kd = 0,0
              MotionCommander.start_linear_motion(velocity_x_m=z*kp/x/y, velocity_y_m=x*kp*-1, velocity_z_m=y*-1*kp)
          else:
              MotionCommander.start_circle_left(radius_m=.1, velocity=.3)




      else:
          with open("img.jpeg", "wb") as f:
              f.write(imgStream)
          nparr = np.frombuffer(imgStream, np.uint8)
          decoded = cv2.imdecode(nparr,cv2.IMREAD_UNCHANGED)
          cv2.imshow('JPEG', decoded)
          cv2.waitKey(1)

