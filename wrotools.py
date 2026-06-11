# WRO TOOLS FILE
# version: 1.0.2
# date: 9/5/2026


import umath

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, run_task, multitask

# MATH STUFF

pi: float = umath.pi

# ROBOT HARDWARE DETAILS

wheel_diameter: float = 68.8
wheel_circumference: float = wheel_diameter * pi
distance_between_wheels: int = 200

# INITIALIZATION            

hub: PrimeHub = PrimeHub()
left_motor: Motor = Motor(Port.D)
right_motor: Motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
color_sensor1: ColorSensor = ColorSensor(Port.C)
attachment_left: Motor = Motor(Port.E)
attachment_right: Motor = Motor(Port.A)
db: DriveBase = DriveBase(left_motor, right_motor, wheel_diameter, distance_between_wheels)
db.use_gyro(False)
watch = StopWatch()
watch.reset()
hub.imu.reset_heading(0)

# HELPER FUNCTIONS



async def resetDB() -> None:
    """
    Resets the driving base
    """

    db.reset()
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    hub.imu.reset_heading(0)
    print("reset complete")
    await wait(50)

def mmToDegrees(mm: int) -> float:
    """
    Converts mm distance to a degrees value for the motors to move
    
    :param mm: The distance in mm being converted
    :type mm: int
    :return: The converted degrees value of the mm value
    :rtype: float
    """

    return (mm/wheel_circumference) * 360

def convertSpeed(speed: float, isLargeMotor: bool) -> float:
    """
    Converts percentage speed to degrees per second
    
    :param speed: The percentage speed being converted
    :type speed: int
    :param isLargeMotor: Controls whether the motor is a large or medium motor
    :type isLargeMotor: bool
    :return: The converted degrees per second measure of the percentage speed
    :rtype: float
    """

    if isLargeMotor:
        return (speed/100) * 1050
    else:
        return (speed/100) * 111


async def moveAttachmentArms(speed, angle):
    speed1 = convertSpeed(-speed, True)
    speed2 = convertSpeed(speed, True)
    

    async def move_right():
        await attachment_right.run_angle(speed1, angle)
        
    async def move_left():
        await attachment_left.run_angle(speed2, angle)
        
    
    await multitask(move_right(), move_left())

def waitForColor(reflection: int):
    while color_sensor1.reflection() > 20:
        wait(10)

def moveUntilColor(desired_color, speed):
    deg_per_second = convertSpeed(speed, isLargeMotor=True)
    speed_mm_s = (deg_per_second / 360.0) * wheel_circumference
    db.drive(speed_mm_s, 0)
    waitForColor(desired_color)
    db.brake()

def detectColor(sensor: ColorSensor):
    values = [sensor.reflection(), sensor.hsv(), sensor.color()]
    print(f"Reflection = {values[0]}, HSV = {values[1]}, Color = {values[2]}")
    return values

"""
def startMovingAtSpeeds(speed1: float, speed2: float) -> None:
    
    Starts moving both motors at individually controlled speeds
    
    :param speed1: Speed value of the left motor
    :type speed1: inted)
        wait(10)

    :param speed2: Speed value of the right motor
    :type speed2: intfrom wrotools import *
    

    left_motor.run(speed1)
    right_motor.run(speed2)


def startDCAtSpeeds(speed1: float, speed2: float) -> None:
    
    Starts individually moving the motors with motor.dc()
    
    :param speed1: Speed value of the left motor
    :type speed1: float
    :param speed2: Speed value of the right motor
    :type speed2: float
    
    left_motor.dc(speed1)
    right_motor.dc(speed2)
    
"""









