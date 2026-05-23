# Legacy movement code
# version: LEGACY-1.0.1
# date: 9/5/2026


import umath

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

# MATH STUFF

pi: float = umath.pi

# ROBOT HARDWARE DETAILS

wheel_diameter: float = 68.8
wheel_circumference: float = wheel_diameter * pi

# INITIALIZATION            

hub: PrimeHub = PrimeHub()
left_motor: Motor = Motor(Port.F, Direction.COUNTERCLOCKWISE)
right_motor: Motor = Motor(Port.B)
#color_sensor1: ColorSensor = ColorSensor(Port.C)
# color_sensor2: ColorSensor = ColorSensor(Port.D)
attachment_left: Motor = Motor(Port.E)
attachment_right: Motor = Motor(Port.A)
watch = StopWatch()
watch.reset()
hub.imu.reset_heading(0)
coordinates: list[float] = [0.0, 0.0]

current_heading_offset = 0.0




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

def resetDB() -> None:
    """
    Resets the driving base
    """

    #db.reset()
    hub.imu.reset_heading(0)
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
        return (speed/100) * 1110

def gyroStraight(min_speed: float, target_distance: float, backwards: bool) -> None:

    """
    Makes the robot move straight while utilizing a PID control system to remain at a yaw angle of 0
    
    :param min_speed: The minimum speed the robot will move at
    :type min_speed: float
    :param target_distance: The distance the robot will move in mm
    :type target_distance: int
    :param backwards: Whether the bot is moving backwards
    :type backwards: bool
    """

    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    

    def kpControl(base_speed: float, target_distance: float) -> tuple[float, float, float]:

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

        left_speed, right_speed, correction = kpControl(speed, target_distance)

        left_speed,right_speed = [-left_speed if backwards else left_speed, -right_speed if backwards else right_speed]


        startDCAtSpeeds(left_speed, right_speed)

        wait(10)

        print(f'Distance: {current_distance}, Angle: {correction}, Speed: {speed}')

    left_motor.hold()
    right_motor.hold()

    dist_moved = -target_distance if backwards else target_distance
    #updateCoordinates(dist_moved)


def gyroTurn(target_angle: float, turn_speed: int) -> None:

    """
    Turns the robot to a target angle using the gyro sensor
    
    :param target_angle: The angle the robot will turn to
    :type target_angle: int
    :param turn_speed: The speed the robot will turn at
    :type turn_speed: int
    :param clockwise: Whether the robot will turn clockwise or counterclockwise
    :type clockwise: bool
    """

    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    
    while abs(hub.imu.heading()) < abs(target_angle) - 1:
        correction = target_angle - hub.imu.heading()
        startDCAtSpeeds(turn_speed + correction, turn_speed - correction)

    left_motor.hold()
    right_motor.hold()



def raedGyroTurn(target_angle: float, max_speed: int) -> None:
    # Use a simple Proportional gain (Kp)
    # Adjust this value (0.5 - 2.0) if the turn is too jerky or too slow
    turn_kp = 1.5 
    
    while True:
        # Calculate how far we are from the target
        error = target_angle - hub.imu.heading()
        
        # If we are within 1 degree, stop
        if abs(error) <= 1:
            break
            
        # Calculate speed based on error
        speed = error * turn_kp
        
        # Ensure speed doesn't exceed your limit, but stays above 
        # a 'minimum' to prevent stalling (e.g., 12%)
        if speed > 0:
            speed = max(12, min(speed, max_speed))
        else:
            speed = min(-12, max(speed, -max_speed))
            
        # Pivot: Left motor moves opposite of Right motor
        startDCAtSpeeds(speed, -speed)
        wait(10)

    left_motor.hold()
    right_motor.hold()