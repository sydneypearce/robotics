#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import AnalogSensor, UARTDevice

# CONSTANTS
con_runtime = 1500
con_runspeed = -125
disk_runspeed = -200
cup_runtime = 500
sort_speed = 100
dist = 45

ev3 = EV3Brick()
sort_motor = Motor(Port.A)
con_motor = Motor(Port.C)
disk_motor = Motor(Port.D)
sensor = UltrasonicSensor(Port.S2)
red_angle = 0
other_angle = -55

uart = UARTDevice(Port.S1, 9600, timeout=2000)


# FUNCTIONS
# called when ultrasonic sensor senses a block
# tells pi to take picture, then calls correct function depending on
# whether or not the brick is red
def take_picture():
    uart.write("1".encode())
    while True:
     if uart.waiting() != 0:
         red = uart.read(1).decode("utf-8") 
         if (red == "1"):
             print("its red")
             isRed()
             return
         elif (red == "0"):
             print("its not red")
             notRed() 
             return

# called if rasppi says piece is red
# turns cup so that red bin is under the belt, turns belt
def isRed(): 
    sort_motor.run_target(sort_speed, red_angle, Stop.COAST, True)
    con_motor.run_time(con_runspeed, con_runtime*2)
    return

# called if rasppi says piece is not red
# turns cup so that other bin is under the belt, turns belt
def notRed():
    sort_motor.run_target(-1*sort_speed, other_angle, Stop.COAST, True)
    con_motor.run_time(con_runspeed, con_runtime*2)
    return

# start
ev3.speaker.beep()
##MAIN
while True:
    disk_motor.run(disk_runspeed)
    if (sensor.distance() > dist):
        con_motor.run_time(con_runspeed, con_runtime, Stop.COAST, True)
        take_picture()

