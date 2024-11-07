from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis
from pybricks.pupdevices import Motor
from pybricks.parameters import Port,Direction
from pybricks.tools import wait

fan_motor = Motor(Port.C, Direction.CLOCKWISE, gears=[40,8,12,12,24,16,8,24], reset_angle=True)

fan_motor.run(5000)
wait(10000)