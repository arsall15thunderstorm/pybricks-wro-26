# WRO TOOLS FILE
# version: 1.2
# date: 15/6/2026


import gc

import umath
from pybricks.hubs import PrimeHub
from pybricks.parameters import Direction, Port, Color
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, multitask

# MATH STUFF

pi: float = umath.pi

# ROBOT HARDWARE DETAILS

wheel_diameter: float = 68.8
wheel_circumference: float = wheel_diameter * pi
distance_between_wheels: int = 205
cleanedList: list[Color] = []
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
watch: StopWatch = StopWatch()
watch.reset()
hub.imu.reset_heading(0)
validColors = [Color.RED, Color.BLUE, Color.GREEN, Color.BLACK, Color.YELLOW]



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
    :type speed: int, %
    :return: The converted degrees per second measure of the percentage speed
    :rtype: Number, deg/st
    """
    return (speed/100) * 1050



async def moveAttachmentArms(speed: float, angle: int) -> None:
    """
    Moves both the attachment arms at the same time

    :param speed: The percentage speed that the arms will move at
    :type speed: int, %
    :param angle: The angle the arms will move by
    :type angle: int, deg
    """

    speed = convertSpeed(speed)

    async def move_right():
        await attachment_right.run_angle(speed, -angle)
        
    async def move_left():
        await attachment_left.run_angle(speed, angle)
        
    
    await multitask(move_right(), move_left())

async def moveRightArm(speed: float, angle: int) -> None:
    """
    Moves both the attachment arms at the same time

    :param speed: The percentage speed that the arms will move at
    :type speed: int, %
    :param angle: The angle the arms will move by
    :type angle: int, deg
    """

    speed = convertSpeed(speed)

    async def move_right():
        await attachment_right.run_angle(speed, -angle)
    await move_right()

async def moveLeftArm(speed: float, angle: int) -> None:
    """
    Moves both the attachment arms at the same time

    :param speed: The percentage speed that the arms will move at
    :type speed: int, %
    :param angle: The angle the arms will move by
    :type angle: int, deg
    """

    speed = convertSpeed(speed)

    async def move_left():
        await attachment_left.run_angle(speed, angle)
    await move_left()

async def moveUntilColor(reflection: int, speed: int, distance: int = 0, use_distance: bool = False) -> None:
    """
    Makes the robot move until either:
    - A. It reaches a color with a reflection below a certain threshold
    - B. A certain distance is reached

    :param reflection: The reflection threshold where the robot will stop moving
    :type reflection: int, %
    :param speed: The percentage speed that the bot will move at
    :type speed: int, %
    :param distance: The secondary distance threshold where the robot will stop
    :type distance: int, mm
    :param use_distance: Controls whether to use the distance check
    :type use_distance: bool
    """

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

    if use_distance:
        await multitask(driveForever(), waitForColor(), detectDistance(), race=True)
    else:
        await multitask(driveForever(), waitForColor(), race=True)

    db.brake()


async def async_wrapper(func, *args, **kwargs):
    """
    Forces a pybricks MaybeAwaitable function to always behave like a coroutine so that it functions with the multitask() function

    :param func: The method to execute.
    :type func: Callable[..., Awaitable[Any]]
    :param args: Positional arguments for the method
    :type args: Any
    :param kwargs: Keyword arguments for the method
    :type kwargs: Any
    :return: The resolved value from the awaited method
    :rtype: Any
    """

    return await func(*args, **kwargs)

async def yellowTowers() -> None:
    """
    Running the first task, which includes:
    - Start calibration
    - Picking up both yellow towers
    - Moving and placing the tower tops on the bases
    """

    db.settings(270, 800, 150, 300)

    # calibration
    await multitask(async_wrapper(db.straight, -500), moveAttachmentArms(40, 450))

    # picking up the towers
    db.settings(200,600,120,300)
    await db.straight(260)
    await db.turn(-90)
    db.stop()
    db.settings(210,400,150,300)
    await db.straight(200)
    db.settings(150,300,120,300)
    await db.straight(110)
    db.settings(250, 650, 150, 300)
    await moveAttachmentArms(40, -390)



    # placing first tower
    db.settings(500,650,150,300)
    await db.straight(-35)
    await db.turn(90)
    await db.straight(503)
    await moveUntilColor(20, 45, 100) #fill distance properly
    db.settings(240, 700, 120, 250)
    await db.straight(430)
    await moveAttachmentArms(40,255)
    await db.straight(-200)
    db.settings(280, 800, 160, 300)

    # calibration
    await db.turn(90)
    await db.straight(-300)


    # placing second tower
    db.settings(450,600,150,300)
    await moveUntilColor(20, 40, 100) # fill distance properly
    await multitask(async_wrapper(db.straight, 328), moveAttachmentArms(40, -250))
    db.settings(240, 700, 120, 250)
    await db.turn(-90)
    await db.straight(210)
    await moveAttachmentArms(38,255)
    await db.straight(-100)
    db.settings(280, 800, 160, 300)



    gc.collect()


async def colorScanning() -> list[Color]:
    """
    Scans colors (of artifacts) until a list of 4, unique, valid (as defined by list validColors) is formed

    :return: The list of scanned colors
    :rtype: list[Color]

    """
    cleanedList = []
    black_debounce_count = 0

    while True:
        currentReflection = await color_sensor2.reflection()
        currentScan = await color_sensor2.color()
        currentHSV = await color_sensor2.hsv()
        finalDebounce = 3
        if 3 <= currentReflection <= 10:
            black_debounce_count += 1
            if black_debounce_count >= finalDebounce:
                if Color.BLACK not in cleanedList:
                    cleanedList.append(Color.BLACK)
                    print(Color.BLACK, currentReflection, currentHSV)
        elif currentScan in validColors:
            black_debounce_count = 0
            if currentScan not in cleanedList:
                cleanedList.append(currentScan)
                print(currentScan, currentReflection, currentHSV)
        else:
            black_debounce_count = 0

        if len(cleanedList) == 4:
            print(cleanedList)
            break

        await wait(50)
    print(cleanedList[3])
    return cleanedList
    