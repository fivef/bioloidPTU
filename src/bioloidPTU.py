import serial
import math

# definition of servo ids
panServoID = 2
tiltServoID = 6

movingSpeed = 100

# register addresses (don't change)
AX12_GOAL_POSITION_L = 30  # set position
AX12_MOVING_SPEED_L = 32  # set speed [1,0x3ff]

# important AX-12 constants
AX_WRITE_DATA = 3
AX_READ_DATA = 4


s = serial.Serial()               # create a serial port object
s.baudrate = 1000000              # baud rate, in bits/second
s.port = "/dev/ttyUSB0"           # this is whatever port your are using
s.open()



def setPanAngleRadian(panAngleRadian):
    panPosition = int(radianAngleToBioloid(panAngleRadian))
    setReg(panServoID,AX12_MOVING_SPEED_L,((movingSpeed%256),(movingSpeed>>8)))
    setReg(panServoID,AX12_GOAL_POSITION_L,((panPosition%256),(panPosition>>8)))
    
    print "Set pan angle: " + str(panAngleRadian) + " (bioloid position: " + str(panPosition) + ")"
    print "Pan servo temperature" + str(getReg(panServoID,43,1)) 

def setTiltAngleRadian(tiltAngleRadian):
    tiltPosition = int(radianAngleToBioloid(tiltAngleRadian))
    setReg(tiltServoID,AX12_MOVING_SPEED_L,((movingSpeed%256),(movingSpeed>>8)))
    setReg(tiltServoID,AX12_GOAL_POSITION_L,((tiltPosition%256),(tiltPosition>>8)))
    
    print "Set tilt angle: " + str(tiltAngleRadian) + " (bioloid position: " + str(tiltPosition) + ")"
    
    print "Tilt servo temperature " + str(getReg(tiltServoID,43,1))

# set register values
def setReg(ID,reg,values):
    length = 3 + len(values)
    checksum = 255-((ID+length+AX_WRITE_DATA+reg+sum(values))%256)          
    s.write(chr(0xFF)+chr(0xFF)+chr(ID)+chr(length)+chr(AX_WRITE_DATA)+chr(reg))
    for val in values:
        s.write(chr(val))
    s.write(chr(checksum))

def getReg(index, regstart, rlength):
    s.flushInput()   
    checksum = 255 - ((6 + index + regstart + rlength)%256)
    s.write(chr(0xFF)+chr(0xFF)+chr(index)+chr(0x04)+chr(AX_READ_DATA)+chr(regstart)+chr(rlength)+chr(checksum))
    vals = list()
    s.read()   # 0xff
    s.read()   # 0xff
    s.read()   # ID
    length = ord(s.read()) - 1
    s.read()   # toss error    
    while length > 0:
        vals.append(ord(s.read()))
        length = length - 1
    if rlength == 1:
        return vals[0]
    return vals

def bioloidToRadianAngle(bioloidPosition):
    angleRadian= ((5*math.pi*bioloidPosition)/3066)
    #Bioloid-Offset: 150 = initial position.
      
    return angleRadian - (math.pi*5/6);

def radianAngleToBioloid(radianAngle):
    #Bioloid-Offset: 150 = initial position.
    radianAngle = radianAngle + (math.pi*5/6);
    return  rRound(((radianAngle/math.pi)*(3066/5)),5)


def rRound(res, dec):
    for i in range(dec-1, 0 ,-1):
        tmp = res*math.pow(10, i)
        if (tmp-math.floor(tmp))>=0.5:
            res=res+math.pow(10,i*(-1))
    return res


setPanAngleRadian(0) #centered position = 0   #negative values pan right
setTiltAngleRadian(-0.85) #centered position = 0  #negative values tilt down 


