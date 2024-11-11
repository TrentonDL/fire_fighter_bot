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
    DESIRED_DISTANCE = 100  # Desired distance from the wall in mm
    min_distance = 80          # Minimum distance threshold from the wall
    max_distance = 120           # Maximum distance threshold from the wall
    speed_rate = 100             # Forward speed_rate in mm/s
    turn_rate = 15          # Turn rate for small adjustments
    goal = False
    # Start moving forward
    drive_base.drive(speed_rate, 0) 
    while not goal:
        # Measure distance to the wall on the left side
        distance = s_ultra.distance()

        # Check proximity to the wall and adjust direction
        if distance < min_distance:
            # Too close to the wall, turn slightly right
            drive_base.drive(speed_rate, turn_rate)
        elif distance > max_distance:
            # Too far from the wall, turn slightly left
            drive_base.drive(speed_rate, -turn_rate)
        else:
            # At the desired distance, go straight
            drive_base.drive(speed_rate, 0)

        # Check for fire color detection to trigger goal behavior
        if c_sensor.color(surface=True) == Color.GREEN:
            goal_found(fan_motor)
            break
