# run using ---->     pybricksdev run ble --name  "Team5"  main_bot.py

from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis,Button
from pybricks.pupdevices import Motor,ColorSensor,UltrasonicSensor
from pybricks.parameters import Port,Direction,Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from urandom import randint
import umath

DEBUG = 1

GOAL = False

def E_STOP_Activated(hub:PrimeHub):
    return Button.CENTER in hub.buttons.pressed()

def wander_area(drive_base=DriveBase,c_sensor=ColorSensor,s_ultra=UltrasonicSensor,f_ultra=UltrasonicSensor, fan_motor=Motor, hub = PrimeHub):
    global GOAL
    if DEBUG:
        print("wander")
    #wander until find wall
    while not GOAL:
        f_distance = f_ultra.distance()
        if DEBUG:
            print(f"f_dist = {f_distance}\n")
        rand_forw_dist = randint(100,305)
        if rand_forw_dist < f_distance: #if the random distance is less than the wall in front; can subtract distance to make it stop sooner
            drive_base.straight(distance=rand_forw_dist)
            
        if check_goal(c_sensor):
            GOAL = True
            goal_found(drive_base, fan_motor, hub)
        
        drive_base.turn(angle = (-1 * randint(10,45))) #rand turn dist from 10 - 45 degrees to the right
        
        s_dist = s_ultra.distance()

        if s_dist < 220: #hi there
            alt_wall_follow(drive_base,c_sensor,s_ultra,f_ultra, fan_motor, hub) #a comment
        elif f_ultra.distance() < 220: #if wall close in front, then turn left
            drive_base.turn(angle=90, wait=True) #fake comment
            alt_wall_follow(drive_base,c_sensor,s_ultra,f_ultra, fan_motor, hub) #fake comment

def alt_wall_follow(drive_base=DriveBase,c_sensor=ColorSensor,s_ultra=UltrasonicSensor,f_ultra=UltrasonicSensor, fan_motor=Motor, hub=PrimeHub):
    global GOAL
    if DEBUG:
        print("wall follow")
    #follow a wall on the robots left side
    drive_base.drive(speed=50, turn_rate=0)
    next_to_wall = True
    dist_increment = 100 #how far the robot will go every cycle to look for a wall in front or adjust to straighten along the wall to the left
    
    #drive forward the dist_increment, if goal is found then end, if wall no longer there then wander
    #else get new distance to wall and try to calculate the angle to straighten along
    while (not GOAL) and next_to_wall:
        front_wall_dist = f_ultra.distance()
        if DEBUG:
            print(f"front wall = {front_wall_dist}\n")
        #if the wall is too close we need to turn right
        if front_wall_dist < dist_increment:
            drive_base.turn(angle=90, wait=True)            
        else:    
            left_wall_dist = s_ultra.distance()
            if DEBUG:
                print(f"left wall = {left_wall_dist}\n")
            drive_base.straight(distance=dist_increment,wait=True)
            new_wall_dist = s_ultra.distance()
            if new_wall_dist < 42 and front_wall_dist > 1999:
                drive_base.straight(distance=-dist_increment, wait=True)
                drive_base.turn(angle=10,wait=True)
            else:
                if check_goal(c_sensor):
                    GOAL = True
                    break
                if new_wall_dist > 200: #wall no longer found
                    next_to_wall = False
                else:
                    #adjust angle to try to drive parallel to the wall
                    dist_difference = new_wall_dist - left_wall_dist
                    adjust_angle_rad = umath.atan2(dist_difference, dist_increment) #debated on acos here?
                    adjust_angle_deg = umath.degrees(adjust_angle_rad)
                    drive_base.turn(angle=adjust_angle_deg)

                    # if left_wall_dist > new_wall_dist:
                    #     drive_base.turn(angle=adjust_angle_deg) # turn right
                    # elif left_wall_dist < new_wall_dist:
                    #     drive_base.turn(angle=-adjust_angle_deg) # turn left
                    # else:
                    #     drive_base.turn(angle=adjust_angle_deg) # will go straight since angle is 0
        
    if GOAL:
        goal_found(drive_base,fan_motor, hub)
    else:
        wander_area(drive_base, c_sensor, s_ultra, f_ultra, fan_motor, hub)

def goal_found(d_base = DriveBase, fan_motor = Motor, hub = PrimeHub):
    d_base.stop()
    # goal Found Siren -> He's a Pirate
    hub.speaker.play_notes(["A3/6","A3/6","A3/6","B3/8","A3/8","R/8","G3/6","G3/6","G3/6","G3/8","A3/8","R/8","A3/6","A3/6","A3/6","B3/8","A3/8","R/8","A3/8","G3/8","E3/8","D3/8"],tempo=120)
    fan_motor.run(10000)
    wait(5000)
    fan_motor.run(0)

def check_goal(c_sensor):
    global GOAL

    color = c_sensor.color(surface=True)

    if DEBUG:
        print(color)
    if color == Color.GREEN:
            GOAL = True
    return GOAL

def clean_Motors(lMotor=Motor,rMotor=Motor,fMotor=Motor,cSenor=ColorSensor,sUltra=UltrasonicSensor,fUltra=UltrasonicSensor):
    lMotor.close()
    rMotor.close()
    fMotor.close()
    cSenor.lights.off()
    sUltra.lights.off()
    fUltra.lights.off()

def E_STOP(lMotor=Motor,rMotor=Motor,fanMotor=Motor, hub=PrimeHub):
    lMotor.stop()
    rMotor.stop()
    fanMotor.stop()
    hub.system.shutdown()

def main():
    #Define All Motors and sensors
    l_Motor = Motor(Port.A,Direction.COUNTERCLOCKWISE,reset_angle=True)
    r_Motor = Motor(Port.B, Direction.CLOCKWISE, reset_angle=True)
    fan_motor = Motor(Port.C, Direction.CLOCKWISE, reset_angle=True)
    color_sensor = ColorSensor(Port.E)
    side_ultra_sonic = UltrasonicSensor(Port.F)
    front_ultra_sonic = UltrasonicSensor(Port.D)

    try:
        #initialize hub and reset IMU
        hub = PrimeHub(top_side=Axis.Z, front_side=Axis.X)
        hub.imu.reset_heading(0)
        
        #starting Siren -> Hoist the Colors
        #hub.speaker.play_notes(["F3/2","B4/2","F3/2","B4/4","B4/4","B4/2","R/8","F3/2","F3/2","A3/2","G3/2","F3/2","G3/1","R/8","G3/2","C3/2","G3/2","G3/2","C3/8","C3/1","R/8","F3/2","G3/2","C3/2","D3/2","E3/1"],tempo=120)

        #initilize Drive Base
        drive_base = DriveBase(l_Motor,r_Motor,wheel_diameter=55.5, axle_track=127)
        drive_base.settings(straight_speed=120 ,turn_rate=25 )
        drive_base.use_gyro(True)

        alt_wall_follow(drive_base,color_sensor,side_ultra_sonic,front_ultra_sonic,fan_motor, hub)
    finally:
        if E_STOP_Activated(hub):
            print("EMERGENCY STOP")
            E_STOP(l_Motor, r_Motor, fan_motor, hub)
        clean_Motors(l_Motor,r_Motor,fan_motor,color_sensor,side_ultra_sonic,front_ultra_sonic)

main()