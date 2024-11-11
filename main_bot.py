# run using ---->     pybricksdev run ble --name  "Team5"  main_bot.py

from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis
from pybricks.pupdevices import Motor,ColorSensor,UltrasonicSensor
from pybricks.parameters import Port,Direction,Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from urandom import randint
import umath

GOAL = False
#follow a wall on the robots left side
MIN_DIST = 30          # Minimum distance threshold from the wall
MAX_DIST = 80           # Maximum distance threshold from the wall
SPEED_RATE = 100             # Forward SPEED_RATE in mm/s
TURN_RATE = 15          # Turn rate for small adjustments

Color.GREEN = Color(h=120, s=100, v=100)
Color.BLUE = Color(h=240, s=94, v=50)
Color.RED = Color(h=0, s=94, v=40)

colors = (Color.GREEN, Color.BLUE, Color.RED)

def wander_area(drive_base=DriveBase,c_sensor=ColorSensor,s_ultra=UltrasonicSensor,f_ultra=UltrasonicSensor, fan_motor=Motor):
    #wander until find wall
    while not GOAL:
        f_distance = f_ultra.distance()
        rand_forw_dist = randint(100,10000)
        if rand_forw_dist < f_distance: #if the random distance is less than the wall in front; can subtract distance to make it stop sooner
            drive_base.straight(rand_forw_dist)
            
        if check_goal(c_sensor):
            GOAL = True
            goal_found(fan_motor)
        
        drive_base.turn(randint(10,90)) #rand turn dist from 10 - 90 degrees to the right
        
        if s_ultra.distance() < 220:
            wall_follow(drive_base=DriveBase,c_sensor=ColorSensor,s_ultra=UltrasonicSensor,f_ultra=UltrasonicSensor, fan_motor=Motor)

def wall_follow(drive_base=DriveBase,c_sensor=ColorSensor,s_ultra=UltrasonicSensor,f_ultra=UltrasonicSensor, fan_motor=Motor):
    # Start moving forward
    drive_base.drive(drive_base,SPEED_RATE, 0) 
    while not GOAL:
        # Measure distance to the wall on the left side
        s_dist = s_ultra.distance()
        f_dist = f_ultra.distance()

        # Check proximity to the wall and adjust direction
        if s_dist < MIN_DIST:
            # Too close to the wall, turn slightly right
            drive_base.drive(SPEED_RATE, TURN_RATE)
        elif s_dist > MAX_DIST:
            # Too far from the wall, turn slightly left
            drive_base.drive(SPEED_RATE, -TURN_RATE)
        elif f_dist < MIN_DIST:
            drive_base.drive(SPEED_RATE, TURN_RATE*5)
        else:
            # At the desired distance, go straight
            drive_base.drive(SPEED_RATE, 0)

        # Check for fire color detection to trigger GOAL behavior
        if c_sensor.color(surface=True) == Color.GREEN:
            goal_found(fan_motor)
            break

def alt_wall_follow(drive_base=DriveBase,c_sensor=ColorSensor,s_ultra=UltrasonicSensor,f_ultra=UltrasonicSensor, fan_motor=Motor):
    #follow a wall on the robots left side
    drive_base.drive(SPEED_RATE,0)
    next_to_wall = True
    dist_increment = int(100) #how far the robot will go every cycle to look for a wall in front or adjust to straighten along the wall to the left
    
    #drive forward the dist_increment, if goal is found then end, if wall no longer there then wander
    #else get new distance to wall and try to calculate the angle to straighten along
    while not GOAL and next_to_wall:
        front_wall_dist = f_ultra.distance()
        #if the wall is too close we need to turn right
        if front_wall_dist < dist_increment:
            drive_base.turn(angle=90, wait=True)            
        else:    
            left_wall_dist = s_ultra.distance()
            drive_base.straight(distance=dist_increment,wait=True)
            new_wall_dist = s_ultra.distance()
            if check_goal(c_sensor):
                GOAL = True
                break
            if new_wall_dist > 100: #wall no longer found
                next_to_wall = False
            else:
                #adjust angle to try to drive parallel to the wall
                dist_difference = new_wall_dist - left_wall_dist
                adjust_angle = umath.atan2(dist_difference,float(250)) #debated on acos here?
                adjust_angle = umath.degrees(adjust_angle)
                drive_base.turn(adjust_angle)
        
    if GOAL:
        goal_found(drive_base,fan_motor)
    else:
        wander_area(drive_base, c_sensor, s_ultra, f_ultra, fan_motor)

def goal_found(d_base = DriveBase,fan_motor=Motor):
    d_base.stop
    fan_motor.run(10000)
    wait(10000)


def check_goal(c_sensor):
    if c_sensor.color(surface=True) == Color.GREEN:
            global GOAL
            GOAL = True
    return GOAL

def clean_Motors(lMotor=Motor,rMotor=Motor,fMotor=Motor,cSenor=ColorSensor,sUltra=UltrasonicSensor,fUltra=UltrasonicSensor):
    lMotor.close()
    rMotor.close()
    fMotor.close()
    cSenor.lights.off()
    sUltra.lights.off()
    fUltra.lights.off()

def main():

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

        #Define All Motors and sensors
        
        drive_base = DriveBase(l_Motor,r_Motor,wheel_diameter=55.5, axle_track=127)
        drive_base.use_gyro(True)

        #goal_found(fan_motor)
        #wander(drive_base, color_sensor, side_ultra_sonic, front_ultra_sonic, fan_motor)
        alt_wall_follow(drive_base,color_sensor,side_ultra_sonic,front_ultra_sonic,fan_motor)
    finally:
        clean_Motors(l_Motor,r_Motor,fan_motor,color_sensor,side_ultra_sonic,front_ultra_sonic)
        


main()