from umath import pi

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, run_task, multitask


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

db.drive(100, 0)
while color_sensor1.reflection() > 20:
    wait(10)
    print(f"{color_sensor1.color()}, {color_sensor1.reflection()}")
db.brake()