#!/usr/bin/env python
import roslib; roslib.load_manifest('bioloidPTU')
import rospy
import serial
import math
import random
import sys
from time import sleep
from std_msgs.msg import Float64
from std_srvs.srv import Empty
import actionlib
from pr2_plugs_msgs.msg import EmptyAction
from pr2_plugs_msgs.msg._EmptyActionResult import EmptyActionResult
from pr2_plugs_msgs.msg._EmptyActionGoal import EmptyActionGoal


# definition of servo ids
PAN_SERVO_ID = 2
TILT_SERVO_ID = 6

EYE_BROWS = 9

MOVE_SPEED = 50

# register addresses (don't change)
AX12_GOAL_POSITION_L = 30  # set position
AX12_MOVE_SPEED_L = 32  # set speed [1,0x3ff]

# important AX-12 constants
AX_WRITE_DATA = 3
AX_READ_DATA = 4

s = serial.Serial()  # create a serial port object

#MAXVALUE LEFT / RIGHT:
maxleftright = 1.40

actionServerLeft = None
actionServerRight = None
actionServerHome = None
actionServerBoss = None
actionServerSad = None
actionServerDart = None
actionServerAround = None

def init():
    global actionServerLeft, actionServerRight, actionServerHome, actionServerCross, actionServerAntiCross, actionServerBoss, actionServerSad, actionServerDart, actionServerAround
 
    rospy.init_node('bioloidPTU', anonymous=True)

    
    s.baudrate = 1000000  # baud rate, in bits/second
    s.port = rospy.get_param('serial_port', '/dev/bioloidptu')
    # this is the serial port your are using, set by parameter
    s.open()

    #rospy.Subscriber("kurtana_pitch_joint_controller/command", Float64, tilt_callback)
    #rospy.Subscriber("kurtana_roll_joint_controller/command", Float64, pan_callback)

    #rospy.Service('robodart_control/look_at_right_magazin', Empty, look_at_right_magazin)
    actionServerLeft = actionlib.SimpleActionServer('robodart_control/look_at_left_magazin', EmptyAction, execute_cb=look_at_left_magazin)
    actionServerRight = actionlib.SimpleActionServer('robodart_control/look_at_right_magazin', EmptyAction, execute_cb=look_at_right_magazin)
    actionServerHome = actionlib.SimpleActionServer('robodart_control/look_at_home', EmptyAction, execute_cb=look_at_home)
    actionServerBoss = actionlib.SimpleActionServer('robodart_control/look_boss', EmptyAction, execute_cb=boss_eyed)
    actionServerSad = actionlib.SimpleActionServer('robodart_control/look_sad', EmptyAction, execute_cb=sad_mode)
    actionServerDart = actionlib.SimpleActionServer('robodart_control/look_at_dartboard', EmptyAction, execute_cb=look_at_dartboard)
    actionServerAround = actionlib.SimpleActionServer('robodart_control/look_around', EmptyAction, execute_cb=look_around)
    actionServerLeft.start()
    actionServerRight.start()
    actionServerHome.start()
    actionServerBoss.start()
    actionServerSad.start()
    actionServerDart.start()
    actionServerAround.start()
    look_at_home_standalone([])
    
    #look_at_right_magazin([])

    rospy.loginfo("bioloidPTU node started")
    rospy.spin()


def set_pan_angle(pan_angle_radian):

    set_servo_angle(PAN_SERVO_ID, pan_angle_radian)


def set_tilt_angle(tilt_angle_radian):

    set_servo_angle(TILT_SERVO_ID, tilt_angle_radian)

def set_servo_angle(servo_id, angle_radian):
    position = int(radian_angle_to_bioloid(angle_radian))
    set_reg(servo_id, AX12_MOVE_SPEED_L, ((MOVE_SPEED % 256), (MOVE_SPEED >> 8)))
    set_reg(servo_id, AX12_GOAL_POSITION_L, ((position % 256), (position >> 8)))

    #sleep(0.1)

    rospy.logdebug("Set servo " + str(servo_id) + " angle: " + str(angle_radian) + " (bioloid position: " + str(position) + ")")


# set register values
def set_reg(servo_id, reg, values):
    length = 3 + len(values)
    checksum = 255 - ((servo_id + length + AX_WRITE_DATA + reg + sum(values)) % 256)
    s.write(chr(0xFF) + chr(0xFF) + chr(servo_id) + chr(length) + chr(AX_WRITE_DATA) + chr(reg))
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

    return angle_radian - (math.pi * 5 / 6)


def radian_angle_to_bioloid(angle_radian):
    # Bioloid-Offset: 150 = initial position.
    angle_radian = angle_radian + (math.pi * 5 / 6);
    return  r_round(((angle_radian / math.pi) * (3066 / 5)), 5)


def r_round(res, dec):
    for i in range(dec - 1, 0, -1):
        tmp = res * math.pow(10, i)
        if (tmp - math.floor(tmp)) >= 0.5:
            res = res + math.pow(10, i * (-1))
    return res

def tilt_callback(data):
    rospy.logdebug("Tilt callback")
    # invert value
    set_tilt_angle(-data.data)
    
def pan_callback(data):
    rospy.logdebug("Pan callback")
    set_pan_angle(data.data)

def look_at_home_standalone(data):
  set_servo_angle(PAN_SERVO_ID, 0)
  set_servo_angle(TILT_SERVO_ID, 0)
  set_servo_angle(EYE_BROWS, 0)


def look_at_home(data):
  set_servo_angle(PAN_SERVO_ID, 0)
  set_servo_angle(TILT_SERVO_ID, 0)
  set_servo_angle(EYE_BROWS, 0)

  
  result = EmptyActionResult()
  actionServerHome.set_succeeded(result=result)


def look_at_right_magazin(data):
  set_servo_angle(PAN_SERVO_ID, -1.7)
  set_servo_angle(TILT_SERVO_ID, 0.45)
  set_servo_angle(EYE_BROWS, -0.3)

  result = EmptyActionResult()
  actionServerRight.set_succeeded(result=result)

def look_at_left_magazin(data):
  set_servo_angle(PAN_SERVO_ID, 1.7)
  set_servo_angle(TILT_SERVO_ID, 0.45)
  set_servo_angle(EYE_BROWS, -0.3)

  
  result = EmptyActionResult()
  actionServerLeft.set_succeeded(result=result) 


def boss_eyed(data):
  for i in range(5):

    brows = random.uniform(-0.1, 0.1)
    tilt  = random.uniform(-0.5, 0.5)
    pan   = random.uniform(-1.5, 1.5)
    sleeptTime = random.uniform(1, 5)
    set_servo_angle(EYE_BROWS, brows)
    set_servo_angle(TILT_SERVO_ID, tilt)
    set_servo_angle(PAN_SERVO_ID, pan)
    sleep(sleeptTime)
  
  result = EmptyActionResult()
  actionServerBoss.set_succeeded(result=result)

def sad_mode(data):
  set_servo_angle(PAN_SERVO_ID, 0)
  set_servo_angle(TILT_SERVO_ID, -0.9)
  set_servo_angle(EYE_BROWS, 0)

  
  sleep(1.0)
  set_servo_angle(PAN_SERVO_ID, 0.5)
  sleep(0.4)
  set_servo_angle(PAN_SERVO_ID, -0.5)
  sleep(0.8)
  set_servo_angle(PAN_SERVO_ID, 0.5)
  sleep(0.8)
  set_servo_angle(PAN_SERVO_ID, -0.5)
  sleep(0.8)
  set_servo_angle(PAN_SERVO_ID, 0.5)
  sleep(0.8)
  
  look_at_home_standalone(None)
  
  result = EmptyActionResult()
  actionServerSad.set_succeeded(result=result)
  
def look_at_dartboard(data):
  set_servo_angle(PAN_SERVO_ID, 0.0)
  set_servo_angle(TILT_SERVO_ID, 0.75)
  set_servo_angle(EYE_BROWS, 0.1)
  
  
  result = EmptyActionResult()
  actionServerDart.set_succeeded(result=result)

def look_around(data):
  while not actionServerAround.is_preempt_requested():
    brows = random.choice([-0.3, 0.0])
    tilt  = random.uniform(-0.3, 0.3)
    pan   = random.uniform(-1.2, 1.2)
    sleeptTime = random.uniform(3, 6)
    set_servo_angle(EYE_BROWS, brows)
    set_servo_angle(TILT_SERVO_ID, tilt)
    set_servo_angle(PAN_SERVO_ID, pan)
    sleep(sleeptTime)
    
  look_at_home_standalone([])
  
  result = EmptyActionResult()
  actionServerAround.set_succeeded(result=result)

    

if __name__ == '__main__':
    
    init()

    """
    # set default args as -h , if no args:
    if len(sys.argv) != 3:
        rospy.logdebug("Usage: bioloidPTU <pan angle in radians> <tilt angle in radians> (default 0 0)")
        set_pan_angle(0)  # centered position = 0   #negative values pan right
        set_tilt_angle(0)  # centered position = 0  #negative values tilt down

    else:
        set_pan_angle(float(sys.argv[1]))  # centered position = 0   #negative values pan right
        set_tilt_angle(float(sys.argv[2]))  # centered position = 0  #negative values tilt down
    """
            
