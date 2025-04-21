from typing import Tuple
import multiprocessing as mp
import argparse
import time
import socket,os,struct, time
import numpy as np
import cv2
import pickle

CHESSBOARD_SIZE : Tuple[int] = (9, 6)  # Define chessboard size
SQUARE_SIZE = 1.0 # Adjust if you know the real size
DECK_IP = '192.168.4.1'
# DECK_IP = "http://24.134.3.9/axis-cgi/mjpg/video.cgi"
DECK_PORT = "5000"
SAVE_PATH = "calibration_images"
SAVE = True

def connect_to_socket(deck_ip=DECK_IP, deck_port=DECK_PORT):
    print("Connecting to socket on {}:{}...".format(deck_ip, deck_port))
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((deck_ip, int(deck_port)))
    print("Socket connected")
    return client_socket

def rx_bytes(socket, size):
    data = bytearray()
    while len(data) < size:
        data.extend(socket.recv(size-len(data)))
    return data

def get_frame(out_queue, stop_event, start):
    client_socket = connect_to_socket()
    if socket is None:
       print("Failed to connect to socket")
       exit(1)

    while not stop_event.is_set():
        # First get the info
        # packetInfoRaw = rx_bytes(4)
        packetInfoRaw = bytearray()
        while len(packetInfoRaw) < 4:
            packetInfoRaw.extend(client_socket.recv(4-len(packetInfoRaw)))
        print("Receiving packet info...")
        print(packetInfoRaw) 
        #print(packetInfoRaw)
        [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)
        #print("Length is {}".format(length))
        #print("Route is 0x{:02X}->0x{:02X}".format(routing & 0xF, routing >> 4))
        #print("Function is 0x{:02X}".format(function))

        imgHeader = rx_bytes(client_socket, length - 2)
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
                packetInfoRaw = rx_bytes(client_socket, 4)
                [length, dst, src] = struct.unpack('<HBB', packetInfoRaw)
                #print("Chunk size is {} ({:02X}->{:02X})".format(length, src, dst))
                chunk = rx_bytes(client_socket, length - 2)
                imgStream.extend(chunk)

            if format == 0:
                bayer_img = np.frombuffer(imgStream, dtype=np.uint8)   
                bayer_img.shape = (244, 324)
                color_img = cv2.cvtColor(bayer_img, cv2.COLOR_BayerBG2BGRA)
                out_queue.put(color_img)
                cv2.imshow('Raw', bayer_img)
                cv2.imshow('Color', color_img)
                cv2.waitKey(1)
            else:
                with open("img.jpeg", "wb") as f:
                    f.write(imgStream)
                nparr = np.frombuffer(imgStream, np.uint8)
                decoded = cv2.imdecode(nparr,cv2.IMREAD_UNCHANGED)
                cv2.imshow('JPEG', decoded)
                cv2.waitKey(1)

def load_camera_matrix(file_path):
    with open(file_path, "rb") as f:
        camera_matrix, dist_coeffs = pickle.load(f)
    return camera_matrix, dist_coeffs



if __name__ == "__main__":
    start_time = time.time()
    # if save path does not exist, create it
    os.makedirs(SAVE_PATH, exist_ok=True)

    camera_matrix, dist_coeffs = load_camera_matrix("camera_calibration.pickle")
    q = mp.Queue(maxsize=10)
    stop_event = mp.Event()
    get_frame_proc = mp.Process(target=get_frame, args=(q, stop_event, start_time))

    get_frame_proc.start()

    try:
        while True:
            frame = q.get(timeout=5)
            # process the frame here
            
            print("Processing frame...", frame)

    except KeyboardInterrupt:
        print("Stopping...")
        stop_event.set()
    finally:
        print("Cleaning up...")
        stop_event.set()
        get_frame_proc.join(5)