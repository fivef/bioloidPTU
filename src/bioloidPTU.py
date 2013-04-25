#!/usr/bin/env python

import serial
import math
import sys


# definition of servo ids
PAN_SERVO_ID = 2
TILT_SERVO_ID = 6

MOVING_SPEED = 100

# register addresses (don't change)
AX12_GOAL_POSITION_L = 30  # set position
AX12_MOVING_SPEED_L = 32  # set speed [1,0x3ff]

# important AX-12 constants
AX_WRITE_DATA = 3
AX_READ_DATA = 4


s = serial.Serial()  # create a serial port object
s.baudrate = 1000000  # baud rate, in bits/second
s.port = "/dev/ttyUSB0"  # this is whatever port your are using
s.open()




def set_pan_angle(pan_angle_radian):
    pan_position = int(radian_angle_to_bioloid(pan_angle_radian))
    set_reg(PAN_SERVO_ID, AX12_MOVING_SPEED_L, ((MOVING_SPEED % 256), (MOVING_SPEED >> 8)))
    set_reg(PAN_SERVO_ID, AX12_GOAL_POSITION_L, ((pan_position % 256), (pan_position >> 8)))
    
    print "Set pan angle: " + str(pan_angle_radian) + " (bioloid position: " + str(pan_position) + ")"
    print "Pan servo temperature " + str(get_reg(PAN_SERVO_ID, 43, 1)) 

def set_tilt_angle(tilt_angle_radian):
    tilt_position = int(radian_angle_to_bioloid(tilt_angle_radian))
    set_reg(TILT_SERVO_ID, AX12_MOVING_SPEED_L, ((MOVING_SPEED % 256), (MOVING_SPEED >> 8)))
    set_reg(TILT_SERVO_ID, AX12_GOAL_POSITION_L, ((tilt_position % 256), (tilt_position >> 8)))
    
    print "Set tilt angle: " + str(tilt_angle_radian) + " (bioloid position: " + str(tilt_position) + ")"
    
    print "Tilt servo temperature " + str(get_reg(TILT_SERVO_ID, 43, 1))

# set register values
def set_reg(id, reg, values):
    length = 3 + len(values)
    checksum = 255 - ((id + length + AX_WRITE_DATA + reg + sum(values)) % 256)          
    s.write(chr(0xFF) + chr(0xFF) + chr(id) + chr(length) + chr(AX_WRITE_DATA) + chr(reg))
    for val in values:
        s.write(chr(val))
    s.write(chr(checksum))

def get_reg(index, regstart, rlength):
    s.flushInput()   
    checksum = 255 - ((6 + index + regstart + rlength) % 256)
    s.write(chr(0xFF) + chr(0xFF) + chr(index) + chr(0x04) + chr(AX_READ_DATA) + chr(regstart) + chr(rlength) + chr(checksum))
    vals = list()
    s.read()  # 0xff
    s.read()  # 0xff
    s.read()  # ID
    length = ord(s.read()) - 1
    s.read()  # toss error    
    while length > 0:
        vals.append(ord(s.read()))
        length = length - 1
    if rlength == 1:
        return vals[0]
    return vals

def bioloid_to_radian_angle(bioloid_position):
    angle_radian = ((5 * math.pi * bioloid_position) / 3066)
    # Bioloid-Offset: 150 = initial position.
      
    return angle_radian - (math.pi * 5 / 6);

def radian_angle_to_bioloid(angle_radian):
    # Bioloid-Offset: 150 = initial position.
    angle_radian = angle_radian + (math.pi * 5 / 6);
    return  r_round(((angle_radian / math.pi) * (3066 / 5)), 5)


def r_round(res, dec):
    for i in range(dec - 1, 0 , -1):
        tmp = res * math.pow(10, i)
        if (tmp - math.floor(tmp)) >= 0.5:
            res = res + math.pow(10, i * (-1))
    return res


if __name__ == '__main__':
    
    # set default args as -h , if no args:
    if len(sys.argv) != 3: 
        print "Usage: bioloidPTU <pan angle in radians> <tilt angle in radians> (default 0 0)"
        set_pan_angle(0)  # centered position = 0   #negative values pan right
        set_tilt_angle(0)  # centered position = 0  #negative values tilt down 
    
    else:
        set_pan_angle(float(sys.argv[1]))  # centered position = 0   #negative values pan right
        set_tilt_angle(float(sys.argv[2]))  # centered position = 0  #negative values tilt down 
        
        
        

   



