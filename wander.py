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

def alt_wall_follow(drive_base=DriveBase,c_sensor=ColorSensor,s_ultra=UltrasonicSensor,f_ultra=UltrasonicSensor, fan_motor=Motor):
    #follow a wall on the robots left side
    drive_base.drive()
    goal = False
    next_to_wall = True
    dist_increment = 250 #how far the robot will go every cycle to look for a wall in front or adjust to straighten along the wall to the left
    
    #drive forward the dist_increment, if goal is found then end, if wall no longer there then wander
    #else get new distance to wall and try to calculate the angle to straighten along
    while not goal and next_to_wall:
        front_wall_dist = f_ultra.distance
        
        #if the wall is too close we need to turn right
        if front_wall_dist <= dist_increment:
            drive_base.turn(90)            
            
        left_wall_dist = s_ultra.distance
        drive_base.straight(distance=dist_increment) #only goes in 250mm stretches, can increase
        new_wall_dist = s_ultra.distance
        if c_sensor.color(surface=True) == Color.GREEN:
            goal = True
        elif new_wall_dist > 100: #wall no longer found
            next_to_wall = False
        else:
            #adjust angle to try to drive parallel to the wall
            dist_difference = new_wall_dist - left_wall_dist
            adjust_angle = umath.atan(dist_difference / 250) #debated on acos here?
            drive_base.turn(adjust_angle)
        
    if goal:
        goal_found(fan_motor)
    else:
        wander(drive_base, c_sensor, s_ultra, f_ultra, fan_motor)