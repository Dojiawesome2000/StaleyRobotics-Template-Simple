from wpilib import *
import wpilib.drive
from wpimath import *
from wpimath.controller import *
# from wpimath.estimator import *
# from wpimath.filter import *
# from wpimath.geometry import *
# from wpimath.interpolation import *
from wpimath.kinematics import *
# from wpimath.spline import *
# # from wpimath.system import *
# from wpimath.trajectory import *
from wpimath.units import *
# from wpinet import *
# from wpiutil import *

# from cscore import *
# from hal import *
# from ntcore import *
# from pyfrc import *
# from robotpy_apriltag import *

# from commands2 import *
# from commands2.button import *
# from commands2.cmd import *

from ctre import *
# from rev import *

class Robot(TimedRobot):
    
    MAX_SPEED = 3.0  # meters per second
    MAX_ANGULAR_SPEED = 2 * math.pi  # one rotation per second

    TRACK_WIDTH = 0.381 * 2  # meters
    WHEEL_RADIUS = 0.0508  # meters
    ENCODER_RESOLUTION = 4096  # counts per revolution

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.frontleft_motor = PWMTalonSRX(0)
        self.backleft_motor = PWMTalonSRX(1)
        self.frontright_motor = PWMTalonSRX(2)
        self.backright_motor = PWMTalonSRX(3)

        self.leftEncoder = wpilib.Encoder(0, 1)
        self.rightEncoder = wpilib.Encoder(2, 3)

        self.left = MotorControllerGroup(self.frontleft_motor, self.backleft_motor)
        self.right = MotorControllerGroup(self.frontright_motor, self.backright_motor)

        self.gyro = wpilib.AnalogGyro(0)

        # creating my kinematics object: track width of 27 units (idk the actual unit type thingy)
        self.kinematics = DifferentialDriveKinematics(self.TRACK_WIDTH)

        # We need to invert one side of the drivetrain so that positive voltages
        # result in both sides moving forward. Depending on how your robot's
        # gearbox is constructed, you might have to invert the left side instead.
        self.right.setInverted(True)

        # Set the distance per pulse for the drive encoders. We can simply use the
        # distance traveled for one rotation of the wheel divided by the encoder
        # resolution.
        self.leftEncoder.setDistancePerPulse(
            2 * math.pi * self.WHEEL_RADIUS / self.ENCODER_RESOLUTION
        )
        self.rightEncoder.setDistancePerPulse(
            2 * math.pi * self.WHEEL_RADIUS / self.ENCODER_RESOLUTION
        )

        self.leftEncoder.reset()
        self.rightEncoder.reset()

        self.odometry = DifferentialDriveOdometry(
            self.gyro.getRotation2d(),
            self.leftEncoder.getDistance(),
            self.rightEncoder.getDistance(),
        )


        self.drive = wpilib.drive.DifferentialDrive(self.left, self.right)
        # self.drive.setExpiration(0.1)

        self.controller = XboxController(0)

        #dunno what this does. Figure it out...
        # self.feedforward = SimpleMotorFeedforwardMeters(1, 3)






    def robotPeriodic(self): pass

    def autonomousInit(self): pass
    def autonomousPeriodic(self): pass
    def autonomousExit(self): pass

    def teleopInit(self):
        """Executed at the start of teleop mode"""
        self.drive.setSafetyEnabled(True)
    def teleopPeriodic(self):
        """Runs the motors with tank steering"""
        self.drive.tankDrive(self.controller.getLeftY() * -1, self.controller.getRightY())
        SmartDashboard.putNumber("TalonSRX front left motor power", self.frontleft_motor.get())
    def teleopExit(self): pass

    def _simulationInit(self): pass
    def _simulationPeriodic(self): pass

    def disabledInit(self): pass
    def disabledPeriodic(self): pass
    def disabledExit(self): pass

if __name__ == '__main__':
    run(Robot)