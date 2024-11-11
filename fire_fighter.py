from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis
from pybricks.pupdevices import Motor
from pybricks.parameters import Port,Direction
from pybricks.tools import wait


def goal_found(fan_motor=Motor):
    fan_motor.run(10000)
    wait(10000)
    
