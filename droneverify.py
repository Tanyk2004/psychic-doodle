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
cube_points = np.float32([
    [0, 0, 0], [cube_size, 0, 0], [cube_size, cube_size, 0], [0, cube_size, 0],  # Bottom face
    [0, 0, -cube_size], [cube_size, 0, -cube_size], [cube_size, cube_size, -cube_size], [0, cube_size, -cube_size]  # Top face
])

camera_matrix = np.array([[190.191, 0, 160.525], [0, 187.639, 147.501], [0, 0, 1]])
dist_coeff = np.array([[-0.08368914,  0.03974283, -0.0016545, 0.0050885,  -0.05173472]])
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size  # Scale if real square size is known
# Create directory to save images
save_path = "calibration_images"
os.makedirs(save_path, exist_ok=True)

while(1):
    # First get the info
    packetInfoRaw = rx_bytes(4)
    #print(packetInfoRaw)
    [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)
    #print("Length is {}".format(length))
    #print("Route is 0x{:02X}->0x{:02X}".format(routing & 0xF, routing >> 4))
    #print("Function is 0x{:02X}".format(function))

    imgHeader = rx_bytes(length - 2)
    #print(imgHeader)
    #print("Length of data is {}".format(len(imgHeader)))
    [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', imgHeader)

    if magic == 0xBC:
      #print("Magic is good")
      #print("Resolution is {}x{} with depth of {} byte(s)".format(width, height, depth))
      #print("Image format is {}".format(format))
      #print("Image size is {} bytes".format(size))

      # Now we start rx the image, this will be split up in packages of some size
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
          bayer_img = np.frombuffer(imgStream, dtype=np.uint8)   
          bayer_img.shape = (244, 324)
          color_img = cv2.cvtColor(bayer_img, cv2.COLOR_BayerBG2BGRA)
          # Detect chessboard corners
          ret, corners = cv2.findChessboardCorners(bayer_img, chessboard_size, None)
          if ret:
            # Refine corners
            corners2 = cv2.cornerSubPix(bayer_img, corners, (11, 11), (-1, -1), (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001))
          
        # Estimate pose (rotation & translation)
            ret, rvec, tvec = cv2.solvePnP(objp, corners2, camera_matrix, dist_coeff)  # No distortion coefficients

            if ret:
            # === Project Cube onto Image ===
                imgpts, _ = cv2.projectPoints(cube_points, rvec, tvec, camera_matrix, dist_coeff)  # No distortion

            # Convert to integer values
                imgpts = np.int32(imgpts).reshape(-1, 2)

            # === Draw Cube ===
            # Bottom square
                color_img = cv2.drawContours(color_img, [imgpts[:4]], -1, (0, 255, 0), 3)
            # Vertical edges
                for i in range(4):
                    color_img = cv2.line(color_img, tuple(imgpts[i]), tuple(imgpts[i + 4]), (255, 0, 0), 3)
            # Top square
                color_img = cv2.drawContours(color_img, [imgpts[4:]], -1, (0, 0, 255), 3)

        # Draw detected corners
            cv2.drawChessboardCorners(color_img, chessboard_size, corners2, ret)
          # Show the color_img
          cv2.imshow("Chessboard Capture", color_img)

          key = cv2.waitKey(1) & 0xFF
          if key == ord("q"):  # Quit when 'q' is pressed
            break
          cv2.waitKey(1)
      else:
          with open("img.jpeg", "wb") as f:
              f.write(imgStream)
          nparr = np.frombuffer(imgStream, np.uint8)
          decoded = cv2.imdecode(nparr,cv2.IMREAD_UNCHANGED)
          cv2.imshow('JPEG', decoded)
          cv2.waitKey(1)

