import logging
import time
from threading import Event, Thread

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils.power_switch import PowerSwitch

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

from cflib.positioning.motion_commander import MotionCommander

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E701'
#TODO: change the uri based on your crazyflie number
'''
1: radio://0/80/2M/E7E7E7E701
2: radio://0/80/2M/E7E7E7E702
3: radio://0/80/2M/E7E7E7E703
4: radio://0/90/2M/E7E7E7E704
5: radio://0/90/2M/E7E7E7E705
6: radio://0/90/2M/E7E7E7E706
7: radio://0/100/2M/E7E7E7E707
8: radio://0/100/2M/E7E7E7E708
9: radio://0/100/2M/E7E7E7E709
'''

DEFAULT_HEIGHT = 0.4

deck_attached_event = Event()

def simple_connect():
    # Test connection. You can use this function to check whether you can connect to crazyflie

    print("Yeah, I'm connected! :D")
    time.sleep(3)
    print("Now I will disconnect :'(")

logging.basicConfig(level=logging.ERROR)


def read_parameter(scf, logconf):
    with SyncLogger(scf, logconf) as logger:
        for log_entry in logger:
            timestamp = log_entry[0]
            data = log_entry[1]
            logconf_name = log_entry[2]

            roll = data['stabilizer.roll']
            pitch = data['stabilizer.pitch']
            yaw = data['stabilizer.yaw']

            print(f"[{timestamp}] Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°")

            break

def moving_up(mc, dis):
    # TODO: move the crazyflie up
    MotionCommander.up(mc, dis, .5)
    # Parameters:
    #       mc: motion commander
    #       dis: a floating number representing move up distance


def moving_down(mc, dis):
    # TODO: move the crazyflie down
    MotionCommander.down(mc, dis, .5)
    # Parameters:
    #       mc: motion commander
    #       dis: a floating number representing move down distance


def forwarding(mc, dis):
    # TODO: move the crazyflie forward
    MotionCommander.forward(mc, dis, .5)
    # Parameters:
    #       mc: motion commander
    #       dis: a floating number representing forward distance


def backwarding(mc, dis):
    # TODO: move the crazyflie backward
    MotionCommander.back(mc, dis, .5)
    # Parameters:
    #       mc: motion commander
    #       dis: a floating number representing backward distance


def turning_left(mc, deg):
    # TODO: turn the crazyflie left
    MotionCommander.turn_left(mc, deg, 20)
    # Parameters:
    #       mc: motion commander
    #       deg: a floating number representing the degree to turn left


def turning_right(mc, deg):
    # TODO: turn the crazyflie right
    MotionCommander.turn_right(mc, deg, 20)
    # Parameters:
    #       mc: motion commander
    #       deg: a floating number representing the degree to turn right


def landing(mc):
    # land the crazyflie
    MotionCommander.land(mc)
    # Parameters:
    #       mc: motion commander


def param_deck_flow(_, value_str):
    # Check whether positioning deck is connected or not

    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')


def fly_commander(scf, lg_stab, mc, SLEEP_TIME = 0.3):
    # Control the crazyflie to following the input command after taking off and before landing
    
    command = ""
    print("input command")
    
    while(command != 'e'):
        command = input()
        command = command.strip()
        if command == 'e':
            break
        elif command == 'i':     # read parameter
            read_parameter(scf, lg_stab)
        elif command[0] == 'u':  # up
            dis = float(command.split()[1])
            moving_up(mc, dis)
        elif command[0] == 'd':  # down
            dis = float(command.split()[1])
            moving_down(mc, dis)
        elif command[0] == 'f':  # forward
            dis = float(command.split()[1])
            forwarding(mc, dis)
        elif command[0] == 'b':  # backward
            dis = float(command.split()[1])
            backwarding(mc, dis)
        elif command[0] == 'l':     # turn left
            deg = float(command.split()[1])
            turning_left(mc, deg)
        elif command[0] == 'r':     # turn right
            deg = float(command.split()[1])
            turning_right(mc, deg)
        elif command == 'n':         # land
            landing(mc)
            return


def take_off_simple(scf, lg_stab):
    print("this is a test")
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        mc.up(DEFAULT_HEIGHT)

def base_commander(scf, lg_stab, DEFAULT_HEIGHT = 0.3, SLEEP_TIME = 0.3):
    # Control the crazyflie to following the input command

    # command = ""
    # mc = None
    # print("input command")
    # while(command != 'e'):
    #     command = input()
    #     command = command.strip()
    #     if command == 'e':
    #         break
    #     elif command == 'i':     # read parameter
    #         read_parameter(scf, lg_stab)
    #     elif command == 's':     # take off
    #         print("crazyflie takes off")
    #         #TODO: let the crazyflie to take off
    #         take_off_simple(scf, lg_stab)
    #         print("past take off")
    #         with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
    #             fly_commander(scf, lg_stab, mc)
    #         #      You can call fly_commander(scf, lg_stab, mc) to deal with flying part
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        fly_commander(scf, lg_stab, mc)
    
def infinite_control(scf, lg_stab):
    while True:
        takeoff_thread = Thread(target=base_commander, args=[scf, lg_stab])
        takeoff_thread.start()
        takeoff_thread.join()
        cmd = input("Press s to take off e to exit")
        if cmd != 's':
            break
        
if __name__ == '__main__':
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
        infinite_control(scf, lg_stab)
