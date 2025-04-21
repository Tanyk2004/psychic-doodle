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
REAL_DIAMETER = 1.75 # cm

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

def get_object_position(frame, camera_matrix, map1, map2, 
        h_min, h_max, s_min, s_max, v_min, v_max, real_diameter = REAL_DIAMETER ):

    
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    undistorted_frame = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)

    # Convert to HSV
    hsv_frame = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2HSV)
    cv2.imshow('HSV', hsv_frame)
    # HSV range for white (adjust if needed)
    # lower_color = np.array([0, 0, 199])
    # upper_color = np.array([179, 8, 255])
    
    lower_color = np.array([h_min, s_min, v_min])
    upper_color = np.array([h_max, s_max, v_max])

    mask = cv2.inRange(hsv_frame, lower_color, upper_color)

    segmented_frame = cv2.bitwise_and(undistorted_frame, undistorted_frame, mask=mask)

    gray = cv2.cvtColor(segmented_frame, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    x, y, z = -1, -1, -1
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        (center, radius) = cv2.minEnclosingCircle(largest_contour)
        center = (int(center[0]), int(center[1]))
        radius = int(radius)

        cv2.circle(undistorted_frame, center, radius, (0, 255, 0), 2)
        cv2.drawContours(undistorted_frame, [largest_contour], -1, (0, 0, 255), 2)

        # 3D position estimate
        px_diameter = 2 * radius
        z = (fx * real_diameter) / px_diameter
        x = (center[0] - cx) * z / fx
        y = (center[1] - cy) * z / fy

        cv2.putText(undistorted_frame, f"Ball at {x:.2f},{y:.2f},{z:.2f}", (center[0], center[1]+20),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

    # Show the result
    cv2.imshow('Undistorted Camera Feed', undistorted_frame)
    return x, y, z


if __name__ == "__main__":
    cv2.namedWindow('mask')
    cv2.createTrackbar('H_min', 'mask', 0, 255,  lambda x: None)
    cv2.createTrackbar('H_max', 'mask', 255, 255, lambda x: None)
    cv2.createTrackbar('S_min', 'mask', 0, 255,  lambda x: None)
    cv2.createTrackbar('S_max', 'mask', 255, 255, lambda x: None)
    cv2.createTrackbar('V_min', 'mask', 0, 255,  lambda x: None)
    cv2.createTrackbar('V_max', 'mask', 255, 255, lambda x: None)

    h_min = 100
    h_max = 140
    s_min = 100
    s_max = 255
    v_min = 100
    v_max = 255

    def save(x):
        h_min = cv2.getTrackbarPos('H_min', 'mask')
        h_max = cv2.getTrackbarPos('H_max', 'mask')
        s_min = cv2.getTrackbarPos('S_min', 'mask')
        s_max = cv2.getTrackbarPos('S_max', 'mask')
        v_min = cv2.getTrackbarPos('V_min', 'mask')
        v_max = cv2.getTrackbarPos('V_max', 'mask')

        with open('color_ranges.txt', 'w') as f:
            f.write(f"{h_min},{h_max},{s_min},{s_max},{v_min},{v_max}") 
    
    def load(x):
        """
        Load the color ranges from the file and set the trackbar positions.
        """
        
        with open('color_ranges.txt', 'r') as f:
            h_min, h_max, s_min, s_max, v_min, v_max = map(int, f.read().split(','))
            cv2.setTrackbarPos('H_min', 'mask', h_min)
            cv2.setTrackbarPos('H_max', 'mask', h_max)
            cv2.setTrackbarPos('S_min', 'mask', s_min)
            cv2.setTrackbarPos('S_max', 'mask', s_max)
            cv2.setTrackbarPos('V_min', 'mask', v_min)
            cv2.setTrackbarPos('V_max', 'mask', v_max)
    
    cv2.setTrackbarPos('H_min', 'mask', 100)
    cv2.setTrackbarPos('H_max', 'mask', 140)
    cv2.setTrackbarPos('S_min', 'mask', 100)
    cv2.setTrackbarPos('S_max', 'mask', 255)
    cv2.setTrackbarPos('V_min', 'mask', 100)
    cv2.setTrackbarPos('V_max', 'mask', 255)

    load(0)

    start_time = time.time()
    # if save path does not exist, create it
    os.makedirs(SAVE_PATH, exist_ok=True)

    camera_matrix, dist_coeffs = load_camera_matrix("camera_calibration.pickle")
    q = mp.Queue(maxsize=10)
    stop_event = mp.Event()
    get_frame_proc = mp.Process(target=get_frame, args=(q, stop_event, start_time))
    w, h = 344, 244
    new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
    map1, map2 = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, new_camera_matrix, (w, h), cv2.CV_16SC2)
    get_frame_proc.start()

    try:
        while True:
            frame = q.get(timeout=5)
            print("Processing frame...", frame)

            # process the frame here
            cv2.imshow("Frame",frame) 
            cv2.waitKey(1)
            # h_min = cv2.getTrackbarPos('H_min', 'mask') 
            # h_max = cv2.getTrackbarPos('H_max', 'mask')
            # s_min = cv2.getTrackbarPos('S_min', 'mask')
            # s_max = cv2.getTrackbarPos('S_max', 'mask')
            # v_min = cv2.getTrackbarPos('V_min', 'mask')
            # v_max = cv2.getTrackbarPos('V_max', 'mask')       

            x, y, z = get_object_position(frame, camera_matrix, 
                map1, map2, h_min, h_max, s_min, s_max, v_min, v_max)
            print(x, y, z)


    except KeyboardInterrupt:
        print("Stopping...")
        stop_event.set()
    finally:
        print("Cleaning up...")
        stop_event.set()
        get_frame_proc.join(5)