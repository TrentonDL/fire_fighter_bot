from pybricks.pupdevices import Motor,ColorSensor,UltrasonicSensor
from pybricks.parameters import Port,Direction,Color
from pybricks.robotics import DriveBase
from fire_fighter import goal_found
from main_bot import colors

def wander(drive_base=DriveBase,c_sensor=ColorSensor,s_ultra=UltrasonicSensor,f_ultra=UltrasonicSensor, fan_motor=Motor):
    #wander until find wall
    drive_base.drive()
    goal = False
    while not goal:
        f_distance = f_ultra.distance
        if f_distance < 1000:
            drive_base.drive(turn_rate=f_distance)
        if c_sensor.color(surface=True) == Color.GREEN:
            goal = True
            break
    
    goal_found(fan_motor)

def wall_follow(drive_base=DriveBase,c_sensor=ColorSensor,s_ultra=UltrasonicSensor,f_ultra=UltrasonicSensor, fan_motor=Motor):
    #follow a wall on the robots left side
    drive_base.drive()
