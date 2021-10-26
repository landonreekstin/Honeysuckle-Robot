# Authors: Michael Brooks, Landon Reekstin    Date: June 10, 2021
# File to setup and run manual control of the robot. Takes gamepad input and 
# controls drive, arm, and spray pump functionality. 


import usb.core
import usb.util
import Jetson.GPIO as GPIO
import sys
import time
import math
from arm_control2 import Arm, Segment
import DriveControl
import PumpControl

# pump variables
pump_pin = 7
pump_input_idx = 6  # 0=off, 2=on

# ----Drive variables----
drive_freq = 1000   # Servo frequency
l_duty_cycle = 50
r_duty_cycle = 50
# PWM pins for driving wheels
left_pin = 32
right_pin = 33
# PS2 input bytearray values
left_input_idx = 2
right_input_idx = 4
# Declare PWM objects
l = GPIO.PWM(left_pin, drive_freq)
r = GPIO.PWM(right_pin, drive_freq)


def driveSetup():  # to be run once at start of program
    # Pin Setup:
    # Board pin-numbering scheme
    GPIO.setmode(GPIO.BOARD)
    # set pin as an output pin with optional initial state
    GPIO.setup(left_pin, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(right_pin, GPIO.OUT, initial=GPIO.HIGH)
    l = GPIO.PWM(left_pin, drive_freq)
    r = GPIO.PWM(right_pin, drive_freq)
    l.start(l_duty_cycle)
    r.start(r_duty_cycle)
    print("Drive PWM Initialized")

def getDriveInput(input_list):
    l_stick_val = input_list[left_input_idx]
    r_stick_val = input_list[right_input_idx]
    l_duty_cycle = -25.4*l_stick_val + 508
    r_duty_cycle = -25.4*r_stick_val + 508

def drive():  # function to be put in main loop
    getDriveInput()
    l.ChangeDutyCycle(l_duty_cycle)
    r.ChangeDutyCycle(r_duty_cycle)

def pumpSetup():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(pump_pin, GPIO.OUT, initial=GPIO.LOW)
    print("Pump initialized")

def updatePumpState(pump_button):
    pump_val = pump_button[pump_input_idx]
    if pump_val == 0:
        GPIO.output(pump_pin, GPIO.LOW)
    elif pump_val == 2:
        GPIO.output(pump_pin, GPIO.HIGH)

def init_input(ret=None):
    # find our device
    dev = usb.core.find(idVendor=0x2563, idProduct=0x0526)

    # was it found?
    if dev is None:
        raise ValueError('Device not found')

    interfaces = dev[0].interfaces()
    for p in interfaces:
        i = p.bInterfaceNumber
        print(i)
        if dev.is_kernel_driver_active(i):
            dev.detach_kernel_driver(i)

            
    # get an endpoint instance
    cfg = dev.get_active_configuration()
    intf = cfg[(0,0)]


    ep = usb.util.find_descriptor(
        intf,
        # match the first OUT endpoint
        custom_match = \
        lambda e: \
            usb.util.endpoint_direction(e.bEndpointAddress) == \
            usb.util.ENDPOINT_IN)
    return ep

def get_input(ep,ret=None):
    inp = ep.read(100)
    print(inp)

    if ret == inp:
        return None
    if inp is None:
        print("failed to get input")
    return inp

def arm_control(inp,arm,dx):
    #parse data
    dPad = inp[5]
    alt_pad = inp[6]

    print('dPad:',dPad)

    sign_y = 0
    sign_z = 0
    sign_x = 0
            
    if dPad != 15:
        if dPad%2: #if input is odd (diagonal)
            pass
        else:            
            if dPad == 0: #forward
                sign_y = 1
            elif dPad == 4: #back
                sign_y = -1
            elif dPad == 2: #right
                sign_x = 1
            elif dPad == 6: #left
                sign_x = -1

    if alt_pad != 0:
        if alt_pad == 16:
            sign_z = 1
        elif alt_pad == 1:
            sign_z = -1

    speed = 2
    
    if inp[8] != 0:
        c = 1.5 * inp[8] / 255
        dx = dx * c
        speed = speed * c
            

    

    #move arm to new position
    if arm.cpos is None:
        arm.get_angles()
    cpos = arm.cpos
        
    y = dx*sign_y + arm.y
    z = dx*sign_z + arm.z
    theta = sign_x * 10* dx + arm.theta
    
    arm.position(y,z,theta,speed,wait=False)

    print(arm.duration)
    time.sleep(arm.duration/1000*0.7)
    
            

def main():
    GPIO.cleanup()  # resets pins
    arm = Arm()
    print('here',arm.duration/1000)
    time.sleep(arm.duration/1000)
    print('done')

    dx = .5

    ep = init_input()
    inp = get_input(ep)
    
    pumpSetup()
    driveSetup()
    
    while True:
        inp = get_input(ep,inp)
        if inp is None:
            time.sleep(.05)
            continue

        if inp[7] == 4:
            arm.reset()
            continue
        print('input:',inp)
        arm_control(inp,arm,dx)
        time.sleep(.1)

        updatePumpState(inp)

        getDriveInput(inp)
        
        
        
if __name__ == "__main__":
    main()

