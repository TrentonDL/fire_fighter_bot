from pybricks.pupdevices import Motor,ColorSensor,UltrasonicSensor
from pybricks.parameters import Port,Direction,Color
from pybricks.robotics import DriveBase
from fire_fighter import goal_found
#from main_bot import colors

#follow a wall on the robots left side
MIN_DIST = 30          # Minimum distance threshold from the wall
MAX_DIST = 80           # Maximum distance threshold from the wall
SPEED_RATE = 100             # Forward SPEED_RATE in mm/s
TURN_RATE = 15          # Turn rate for small adjustments
GOAL = False

def wander_area(drive_base=DriveBase,c_sensor=ColorSensor,s_ultra=UltrasonicSensor,f_ultra=UltrasonicSensor, fan_motor=Motor):
    #wander until find wall
    drive_base.drive()

    while not GOAL:
        f_distance = f_ultra.distance
        if f_distance < MAX_DIST:
            drive_base.drive(SPEED_RATE, TURN_RATE)
        if c_sensor.color(surface=True) == Color.GREEN:
            GOAL = True
    
    goal_found(fan_motor)

def wall_follow(drive_base=DriveBase,c_sensor=ColorSensor,s_ultra=UltrasonicSensor,f_ultra=UltrasonicSensor, fan_motor=Motor):
    # Start moving forward
    drive_base.drive(SPEED_RATE, 0) 
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
            GOAL = True
    
    goal_found(fan_motor)
