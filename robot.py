from wpilib import *
from wpimath import *
from wpimath.controller import *
from wpimath.estimator import *
from wpimath.filter import *
from wpimath.geometry import *
from wpimath.interpolation import *
from wpimath.kinematics import *
from wpimath.spline import *
from wpimath.system import *
from wpimath.trajectory import *
from wpimath.units import *
from wpinet import *
from wpiutil import *

from cscore import *
from hal import *
from ntcore import *
from pyfrc import *
from robotpy_apriltag import *

from commands2 import *
from commands2.button import *
from commands2.cmd import *

from ctre import *
from rev import *

class Robot(TimedRobot):
    def robotInit(self):
        self.left_motor = wpilib.TalonSRX(0)
        self.right_motor = wpilib.TalonSRX(1)
    def robotPeriodic(self): pass

    def autonomousInit(self): pass
    def autonomousPeriodic(self): pass
    def autonomousExit(self): pass

    def teleopInit(self): pass
    def teleopPeriodic(self): pass    
    def teleopExit(self): pass

    def _simulationInit(self): pass
    def _simulationPeriodic(self): pass

    def disabledInit(self): pass
    def disabledPeriodic(self): pass
    def disabledExit(self): pass

if __name__ == '__main__':
    run(Robot)