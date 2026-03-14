# WRO TOOLS FILE
# version: 1.00
# date: 14/3/2026


import math

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

# MATH STUFF

pi = math.pi

# ROBOT HARDWARE DETAILS

wheel_diameter: float = 68.8
wheel_circumference: float = wheel_diameter * pi
distance_between_wheels: int = 200

# INITIALIZATION            

hub = PrimeHub()
left_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.A)
db = DriveBase(left_motor, right_motor, wheel_diameter, distance_between_wheels)
db.use_gyro(True)
watch = StopWatch()
watch.reset()
db.reset()
hub.imu.reset_heading(0)

# CONSTANTS

# will change these as needed
Kp: float = 2 
Ki: float = 0.025
Kd: float = 1.2

target_distance: int = 30
min_speed: int = 37
max_speed: int = 70

# PID STATE VARIABLES

integral: float = 0
lastTime = watch.time()
lastError: float = 0

cal_factor: float = 1 # Will change later

distance_per_rotation: float = wheel_circumference*cal_factor

# DISTANCE SEGMENTS
part1: float = 0.2 * target_distance
part3: float = 0.2 * target_distance



# HELPER FUNCTIONS

def startMovingAtSpeeds(speed1: int, speed2: int) -> None:
    """
    Starts moving both motors at individually controlled speeds
    
    :param speed1: Speed value of the left motor
    :type speed1: int
    :param speed2: Speed value of the right motor
    :type speed2: int
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

def resetDB() -> None:
    """
    Resets the driving base
    """

    db.reset()
    hub.imu.reset_heading(0)
    print("reset complete")
    wait(50)

def mm_to_degrees(mm: int) -> float:
    """
    Converts mm distance to a degrees value for the motors to move
    
    :param mm: The distance in mm being converted
    :type mm: int
    :return: The converted degrees value of the mm value
    :rtype: float
    """

    return (mm/wheel_circumference) * 360

def convert_speed(speed: int, isLargeMotor: bool) -> float:
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
        return (speed/100) * 1110

def gyroStraight(min_speed: float, target_distance: int, backwards: bool) -> None:

    """
    Makes the robot move straight while utilizing a PID control system to remain at a yaw angle of 0
    
    :param min_speed: The minimum speed the robot will move at
    :type min_speed: float
    :param target_distance: The distance the robot will move in mm
    :type target_distance: int
    :param backwards: Whether the bot is moving backwards
    :type backwards: bool
    """

    resetDB()

    def kp_control(base_speed: float, target_distance: int) -> tuple[float, float, float]:

        """
        Controls and calculates the PID based motor speeds and correction
        
        :param base_speed: Base speed the robot will move at
        :type base_speed: float
        :param target_distance: The target distamce the robot will move
        :type target_distance: int
        :return: A tuple of the left motor speed, right motor speed, and correction
        :rtype: tuple[float, float, float]
        """


        global integral, lastError, lastTime, derivative
        error = hub.imu.heading()

        current_time = watch.time()

        integral += error
        integral = min(integral, 100)
        dt = current_time - lastTime

        if dt <= 0:
            derivative = 0
        else:
            derivative = (error - lastError)/dt
        
        correction = (Kp * error) + (Ki * integral) + (Kd * derivative)

        lastError = error
        lastTime = current_time

        left_speed = base_speed - correction
        left_speed = max(min(left_speed, 100), -100)
        right_speed = base_speed + correction
        right_speed = max(min(right_speed, 100), -100)

        return left_speed, right_speed, correction
    
    while True:

        left_rotations = abs(left_motor.angle() / 360)
        right_rotations = abs(right_motor.angle() / 360)
        current_distance = (left_rotations + right_rotations) / 2 * distance_per_rotation

        if current_distance >= target_distance:
            break

        
        if current_distance <= part1:
            speed = (max_speed - min_speed) / part1 * current_distance + min_speed
        elif current_distance >= target_distance - part3:
            speed = (min_speed - max_speed) / part3 * (current_distance - (target_distance - part3)) + max_speed
        else:
            speed = max_speed

        left_speed, right_speed, correction = kp_control(speed, target_distance)

        left_speed,right_speed = [-left_speed if backwards else left_speed, -right_speed if backwards else right_speed]


        startDCAtSpeeds(left_speed, right_speed)

        wait(10)

        print(f'Distance: {current_distance}, Angle: {correction}, Speed: {speed}')

    left_motor.hold()
    right_motor.hold()

gyroStraight(10, 200, True)
gyroStraight(10, 200, False)