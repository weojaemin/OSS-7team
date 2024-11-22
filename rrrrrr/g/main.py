#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import *
from pybricks.parameters import *
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import UARTDevice
import time



# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
ser = UARTDevice(Port.S3,baudrate=115200)


# Write your program here.
left_motor=Motor(Port.B)
right_motor=Motor(Port.C)

wheel_diameter = 56
axle_track = 115

robot = DriveBase(left_motor,right_motor,wheel_diameter,axle_track)
gyro=GyroSensor(Port.S4)
grab_motor=Motor(Port.A)
shoot_motor=Motor(Port.D)

def grabMotion1():
    #팔 올리기
    grab_motor.run_until_stalled(-100)
def grabMotion2():
    #중간 높이 팔 올리기
    grabMotion3()
    grab_motor.reset_angle(0)
    grab_motor.run_target(100,-150)

def grabMotion3():
    #팔을 아래로 내리기
    grab_motor.run_until_stalled(100)

def shootMotion1():
    #제일 아래로 초기화하기
    shoot_motor.run_until_stalled(-100)
def shootMotion2():
    #공을 앞으로 던지기
    grabMotion1()
    shoot_motor.run_until_stalled(-2200)


def data_filter(data):
    decoded_data=data.decode().strip()
    if decoded_data.isdigit():
        return int(decoded_data)
    return None

def pd_control(cam_data,kp,kd,power):
    global previous_error
    error = threshold - cam_data
    derivative = error - previous_error
    output = (kp*error)+(kd*derivative)
    robot.drive(power,output)
    previous_error=error

ev3.speaker.beep()
threshold=200
previous_error=0

v=0



while v==1:
    data=ser.read_all()
    print(data)
    time.sleep_ms(1000)


while v==0:
    data = ser.read_all()
    print(data)
    if data:
        filtered_data = data_filter(data)
        if filtered_data is not None:
            print(filtered_data)
            pd_control(filtered_data,kp=0.5,kd=0.1,power=100)
    wait(10)












