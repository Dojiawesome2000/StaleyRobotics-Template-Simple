from wpilib import *
import wpilib.drive
from wpimath import *
from wpimath.controller import *
from wpimath.estimator import *
from wpimath.filter import *
from wpimath.geometry import *
from wpimath.interpolation import *
from wpimath.kinematics import *
from wpimath.spline import *
# from wpimath.system import *
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
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.frontleft_motor = PWMTalonSRX(0)
        self.frontright_motor = PWMTalonSRX(2)
        self.backleft_motor = PWMTalonSRX(1)
        self.backright_motor = PWMTalonSRX(3)

        self.left = MotorControllerGroup(self.frontleft_motor, self.backleft_motor)
        self.right = MotorControllerGroup(self.frontright_motor, self.backright_motor)
        
        self.drive = wpilib.drive.DifferentialDrive(self.left, self.right)
        self.drive.setExpiration(0.1)

        self.controller = XboxController(0)



    def robotPeriodic(self): pass

    def autonomousInit(self): pass
    def autonomousPeriodic(self): pass
    def autonomousExit(self): pass

    def teleopInit(self):
        """Executed at the start of teleop mode"""
        self.drive.setSafetyEnabled(True)
    def teleopPeriodic(self):
        """Runs the motors with tank steering"""
        self.drive.tankDrive(self.controller.getLeftY * -1, self.controller.getRightY)
        SmartDashboard.putNumber("TalonSRX front left motor power", self.frontleft_motor.get())
        SmartDashboard.updateValues()
    def teleopExit(self): pass

    def _simulationInit(self): pass
    def _simulationPeriodic(self): pass

    def disabledInit(self): pass
    def disabledPeriodic(self): pass
    def disabledExit(self): pass

if __name__ == '__main__':
    run(Robot)