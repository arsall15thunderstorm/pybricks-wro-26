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
left_motor: Motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
right_motor: Motor = Motor(Port.B)
color_sensor1: ColorSensor = ColorSensor(Port.C)
attachment_left: Motor = Motor(Port.E)
attachment_right: Motor = Motor(Port.A)
db: DriveBase = DriveBase(left_motor, right_motor, wheel_diameter, distance_between_wheels)
db.use_gyro(True)
watch = StopWatch()
watch.reset()
hub.imu.reset_heading(0)


# HELPER FUNCTIONS

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

def convertSpeed(speed: float) -> float:
    """
    Converts percentage speed to degrees per second
    
    :param speed: The percentage speed being converted
    :type speed: int
    :param isLargeMotor: Controls whether the motor is a large or medium motor
    :type isLargeMotor: bool
    :return: The converted degrees per second measure of the percentage speed
    :rtype: float
    """
    return (speed/100) * 1050



async def moveAttachmentArms(speed, angle):
    speed1 = convertSpeed(-speed)
    speed2 = convertSpeed(speed)
    

    async def move_right():
        await attachment_right.run_angle(speed1, angle)
        
    async def move_left():
        await attachment_left.run_angle(speed2, angle)
        
    
    await multitask(move_right(), move_left())



async def moveUntilColor(reflection, speed):

    async def waitForColor():
        while await color_sensor1.reflection() != reflection:
            await wait(10)

    async def driveForever():
        db.drive(0.6*speed, 0)

        while True:
            await wait(10)
    

    
    await multitask(driveForever(), waitForColor(), race=True)

    db.brake()
 