from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis
from pybricks.pupdevices import Motor,ColorSensor,UltrasonicSensor
from pybricks.parameters import Port,Direction,Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from fire_fighter import goal_found

Color.GREEN = Color(h=120, s=100, v=100)
Color.BLUE = Color(h=240, s=94, v=50)
Color.RED = Color(h=0, s=94, v=40)

colors = (Color.GREEN, Color.BLUE, Color.RED)

from wander import wall_follow, wander_area

from clean_device import *

def main():
    try:
        #initialize hub and reset IMU
        hub = PrimeHub(top_side=Axis.Z, front_side=Axis.X)
        hub.imu.reset_heading(0)

        #Define All Motors and sensors
        l_Motor = Motor(Port.A,Direction.COUNTERCLOCKWISE,reset_angle=True)
        r_Motor = Motor(Port.B, Direction.CLOCKWISE, reset_angle=True)
        fan_motor = Motor(Port.C, Direction.CLOCKWISE, reset_angle=True)
        color_sensor = ColorSensor(Port.E)
        side_ultra_sonic = UltrasonicSensor(Port.F)
        front_ultra_sonic = UltrasonicSensor(Port.D)

        drive_base = DriveBase(l_Motor,r_Motor,wheel_diameter=55.5, axle_track=127)
        drive_base.use_gyro(True)

        #goal_found(fan_motor)
        #wander(drive_base, color_sensor, side_ultra_sonic, front_ultra_sonic, fan_motor)
        wall_follow(drive_base,color_sensor,side_ultra_sonic,front_ultra_sonic,fan_motor)
    finally:
        clean_Motors(l_Motor,r_Motor,fan_motor,color_sensor,side_ultra_sonic,front_ultra_sonic)
        

main()