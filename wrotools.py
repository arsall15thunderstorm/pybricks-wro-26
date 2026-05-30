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
left_motor: Motor = Motor(Port.F, Direction.COUNTERCLOCKWISE)
right_motor: Motor = Motor(Port.B)
color_sensor1: ColorSensor = ColorSensor(Port.C)
attachment_left: Motor = Motor(Port.E)
attachment_right: Motor = Motor(Port.A)
db: DriveBase = DriveBase(left_motor, right_motor, wheel_diameter, distance_between_wheels)
db.use_gyro(True)
watch = StopWatch()
watch.reset()
hub.imu.reset_heading(0)
coordinates: list[float] = [0.0, 0.0]


# HELPER FUNCTIONS


def setCoordinates(x: float, y: float) -> None:
    global coordinates

    coordinates[0] = x
    coordinates[1] = y

def updateCoordinates(distance_mm: float) -> None:
    angle_rad = umath.radians(hub.imu.heading())

    delta_x = distance_mm * umath.cos(angle_rad)
    delta_y = distance_mm * umath.sin(angle_rad)

    coordinates[0] += delta_x
    coordinates[1] += delta_y


def resetDB() -> None:
    """
    Resets the driving base
    """

    db.reset()
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    print("reset complete")
    wait(50)

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

    

def dbMoveWrapper(distance):
    db.straight(distance)
    updateCoordinates(distance)



def moveToCoordinates(target_x: float, target_y: float) -> None:
    
    """
    Moves the robot to a target coordinate using the gyro sensor and odometry
    
    :param target_x: The x coordinate the robot will move to
    :type target_x: float
    :param target_y: The y coordinate the robot will move to
    :type target_y: float
    :param move_speed: The speed the robot will move at
    :type move_speed: int
    """

    angle = umath.degrees(umath.atan2(target_y - coordinates[1], target_x - coordinates[0]))
    distance = umath.sqrt((target_x - coordinates[0])**2 + (target_y - coordinates[1])**2)

    db.turn(angle)
    db.straight(int(distance))
    db.turn(-angle)


async def moveAttachmentArms(speed, angle):
    speed1 = convertSpeed(-speed, True)
    speed2 = convertSpeed(speed, True)
    

    async def move_right():
        await attachment_right.run_angle(speed1, angle)
        
    async def move_left():
        await attachment_left.run_angle(speed2, angle)
        
    
    await multitask(move_right(), move_left())

def waitForColor(desired_color: Color):
    while color_sensor1.hsv() != desired_color:
        wait(20)

def moveUntilColor(desired_color, speed):
    db.drive(100, convertSpeed(speed, True))
    waitForColor(desired_color)
    db.brake()

# not used and will not be used for a while:


def startMovingAtSpeeds(speed1: float, speed2: float) -> None:
    """
    Starts moving both motors at individually controlled speeds
    
    :param speed1: Speed value of the left motor
    :type speed1: inted)
        wait(10)

    :param speed2: Speed value of the right motor
    :type speed2: intfrom wrotools import *
    """

    left_motor.run(speed1)
    right_motor.run(speed2)


def startDCAtSpeeds(speed1: float, speed2: float) -> None:
    """
    Starts individually moving the motors with motor.dc()
    
    :param speed1: Speed value of the left motor
    :type speed1: float
    :param speed2: Speed value of the right motor
    :type speed2: float
    """
    left_motor.dc(speed1)
    right_motor.dc(speed2)








