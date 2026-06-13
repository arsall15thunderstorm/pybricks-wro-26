# WRO TOOLS FILE
# Version 1.1.0
# Date: 13/6/2026

import umath, gc

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
left_motor: Motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
right_motor: Motor = Motor(Port.D)
color_sensor1: ColorSensor = ColorSensor(Port.C)
color_sensor2: ColorSensor = ColorSensor(Port.F)
attachment_left: Motor = Motor(Port.E)
attachment_right: Motor = Motor(Port.A)
db: DriveBase = DriveBase(left_motor, right_motor, wheel_diameter, distance_between_wheels)
db.use_gyro(True)
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
    print("reset complete")
    await wait(50)

def convertSpeed(speed: float) -> float:
    """
    Converts percentage speed to degrees per second
    
    :param speed: The percentage speed being converted
    :type speed: int
    :return: The converted degrees per second measure of the percentage speed
    :rtype: float
    """
    return (speed/100) * 1050



async def moveAttachmentArms(speed, angle):
    speedc = convertSpeed(speed)
    
    

    async def move_right():
        await attachment_right.run_angle(speedc, -angle)
        
    async def move_left():
        await attachment_left.run_angle(speedc, angle)
        
    
    await multitask(move_right(), move_left())



async def moveUntilColor(reflection, speed, distance):

    async def waitForColor():
        while await color_sensor1.reflection() > reflection:
            await wait(10)

    async def driveForever():
        db.drive(0.6004*convertSpeed(speed), 0)

        while True:
            await wait(10)
    
    async def detectDistance():
        while True:
            distance_moved = 0.6004*right_motor.angle()
            if distance_moved >= distance:
                break
            await wait(10)
        
    
    await resetDB()
    
    await multitask(driveForever(), waitForColor(), detectDistance(), race=True)

    db.brake()
 

async def async_wrapper(func, *args, **kwargs):
    return await func(*args, **kwargs)

async def yellowTowers():


    # calibration
    await multitask(async_wrapper(db.straight, -500), moveAttachmentArms(40, 450))
    hub.imu.reset_heading(0)

    # picking up the towers
    await db.straight(256) 
    await db.turn(-90)
    await db.straight(310)
    await moveAttachmentArms(40, -390)

    # placing first tower
    await db.straight(-30)
    await db.turn(90)
    await db.straight(500)
    await moveUntilColor(15, 40, 100) # add distance
    await db.straight(440)
    await moveAttachmentArms(40,260)
    await db.straight(-200)

    # calibration
    await db.turn(90)
    await db.straight(-300)
    hub.imu.reset_heading(0)


    # placing second tower
    await moveUntilColor(15,40, 100) # add distance
    await multitask(async_wrapper(db.straight, 335), moveAttachmentArms(40, -260))
    await db.turn(-90)
    await db.straight(190)
    await moveAttachmentArms(40,260)
    await db.straight(-200)

    gc.collect()
