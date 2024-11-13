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
    s_dist = s_ultra.distance()

    if s_dist < 220:
        alt_wall_follow(drive_base,c_sensor,s_ultra,f_ultra, fan_motor, hub)
    elif f_ultra.distance() < 220: #if wall close in front, then turn left
        drive_base.turn(angle=90, wait=True) 
        alt_wall_follow(drive_base,c_sensor,s_ultra,f_ultra, fan_motor, hub)

    while not GOAL:
        f_distance = f_ultra.distance()
        if DEBUG:
            print(f"f_dist = {f_distance}\n")
        rand_forw_dist = randint(100,305)
        if rand_forw_dist < f_distance-100: #if the random distance is less than the wall in front; can subtract distance to make it stop sooner
            drive_base.straight(distance=rand_forw_dist)
            
        if check_goal(c_sensor):
            GOAL = True
            goal_found(drive_base, fan_motor, hub)
        
        drive_base.turn(angle = (randint(-45,45))) #rand turn dist from 10 - 45 degrees to the right
        
        s_dist = s_ultra.distance()

        if s_dist < 220:
            alt_wall_follow(drive_base,c_sensor,s_ultra,f_ultra, fan_motor, hub)
        elif f_ultra.distance() < 220: #if wall close in front, then turn left
            drive_base.turn(angle=90, wait=True) 
            alt_wall_follow(drive_base,c_sensor,s_ultra,f_ultra, fan_motor, hub)

def alt_wall_follow(drive_base=DriveBase,c_sensor=ColorSensor,s_ultra=UltrasonicSensor,f_ultra=UltrasonicSensor, fan_motor=Motor, hub=PrimeHub):
    global GOAL
    if DEBUG:
        print("wall follow")
    #follow a wall on the robots left side
    drive_base.drive(speed=570, turn_rate=0)
    next_to_wall = True
    dist_increment = 100 #how far the robot will go every cycle to look for a wall in front or adjust to straighten along the wall to the left
    
    #drive forward the dist_increment, if goal is found then end, if wall no longer there then wander
    #else get new distance to wall and try to calculate the angle to straighten along
    while (not GOAL) or next_to_wall:
        front_wall_dist = f_ultra.distance()
        if DEBUG:
            print(f"front wall = {front_wall_dist}\n")
        #if the wall is too close we need to turn right
        if front_wall_dist < (dist_increment + 20) :
            drive_base.turn(angle=90, wait=True)            
        else:    
            left_wall_dist = s_ultra.distance()
            if DEBUG:
                print(f"left wall = {left_wall_dist}\n")
            drive_base.straight(distance=dist_increment,wait=True)
            new_wall_dist = s_ultra.distance()
            if (new_wall_dist < 42 and front_wall_dist > 1999):
                drive_base.straight(distance=-dist_increment, wait=True)
                drive_base.turn(angle=10,wait=True)
            else:
                if check_goal(c_sensor):
                    GOAL = True
                    break
                if new_wall_dist > 200: #wall no longer found
                    next_to_wall = False
                    drive_base.curve(radius=(next_to_wall+110),angle=-120,wait=True)
                    break
                else:
                    #adjust angle to try to drive parallel to the wall
                    dist_difference = new_wall_dist - left_wall_dist
                    adjust_angle_rad = umath.atan2(dist_difference, dist_increment) #debated on acos here?
                    adjust_angle_deg = umath.degrees(adjust_angle_rad)
                    drive_base.turn(angle=-adjust_angle_deg)
    
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
        #hub.speaker.play_notes(["F3/2","B4/2","F3/2","B4/4","B4/4","B4/2","R/8","F3/2","F3/2","A3/2","G3/2","F3/2","G3/1","R/8","G3/2","C3/2","G3/2","G3/2","C3/8","C3/1","R/8","F3/2","G3/2","C3/2","D3/2","E3/1"],tempo=200)

        #initilize Drive Base
        drive_base = DriveBase(l_Motor,r_Motor,wheel_diameter=55.5, axle_track=127)
        drive_base.settings(straight_speed=120 ,turn_rate=25 )
        drive_base.use_gyro(True)
        fan_motor.track_target(20)
        wander_area(drive_base,color_sensor,side_ultra_sonic,front_ultra_sonic,fan_motor, hub)
    finally:
        if E_STOP_Activated(hub):
            print("EMERGENCY STOP")
            E_STOP(l_Motor, r_Motor, fan_motor, hub)
        clean_Motors(l_Motor,r_Motor,fan_motor,color_sensor,side_ultra_sonic,front_ultra_sonic)

main()