from wpilib import *
import wpilib.drive
import wpimath
import wpimath.controller
# from wpimath.estimator import *
# from wpimath.filter import *
# from wpimath.geometry import *
# from wpimath.interpolation import *
import wpimath.kinematics
# from wpimath.spline import *
# # from wpimath.system import *
# from wpimath.trajectory import *
import wpimath.units
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
import ctre
import rev

class Robot(TimedRobot):
    
    MAX_SPEED = 3.0  # meters per second
    MAX_ANGULAR_SPEED = 2 * wpimath.units.math.pi  # one rotation per second

    TRACK_WIDTH = 0.381 * 2  # meters
    WHEEL_RADIUS = 0.0508  # meters
    ENCODER_RESOLUTION = 4096  # counts per revolution

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.frontleft_motor = ctre.WPI_VictorSPX(3)
        self.backleft_motor = ctre.WPI_VictorSPX(4)
        self.frontright_motor = ctre.WPI_VictorSPX(1)
        self.backright_motor = ctre.WPI_VictorSPX(2)

        #Ball launcher go brrrrrrrrrrrrrrr
        self.shooter = rev.CANSparkMax(9, rev.CANSparkMax.MotorType.kBrushless)

        #Reset ball launcher for purposes
        self.shooter.restoreFactoryDefaults()

        #Get PID controller of ball launcher
        self.shooter_pidController = self.shooter.getPIDController()

        #Get encodeer of ball launcher
        self.encoder = self.shooter.getEncoder()

        #Set PID coefficients (to be tampered with)
        self.kP = 6e-5
        self.kI = 0
        self.kD = 0 
        self.kIz = 0
        self.kFF = 0.000015 
        self.kMaxOutput = 1 
        self.kMinOutput = -1
        self.maxRPM = 5700

        # display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", self.kP)
        SmartDashboard.putNumber("I Gain", self.kI)
        SmartDashboard.putNumber("D Gain", self.kD)
        SmartDashboard.putNumber("I Zone", self.kIz)
        SmartDashboard.putNumber("Feed Forward", self.kFF)
        SmartDashboard.putNumber("Max Output", self.kMaxOutput)
        SmartDashboard.putNumber("Min Output", self.kMinOutput)

        # self.leftEncoder = wpilib.Encoder(0, 1)
        # self.rightEncoder = wpilib.Encoder(2, 3)

        self.left = MotorControllerGroup(self.frontleft_motor, self.backleft_motor)
        self.right = MotorControllerGroup(self.frontright_motor, self.backright_motor)

        # self.gyro = wpilib.AnalogGyro(0)

        # creating my kinematics object: track width of 27 units (idk the actual unit type thingy)
        self.kinematics = wpimath.kinematics.DifferentialDriveKinematics(self.TRACK_WIDTH)

        # We need to invert one side of the drivetrain so that positive voltages
        # result in both sides moving forward. Depending on how your robot's
        # gearbox is constructed, you might have to invert the left side instead.
        self.right.setInverted(True)

        self.shooter.setInverted(True)

        # Set the distance per pulse for the drive encoders. We can simply use the
        # distance traveled for one rotation of the wheel divided by the encoder
        # # resolution.
        # self.leftEncoder.setDistancePerPulse(
        #     2 * math.pi * self.WHEEL_RADIUS / self.ENCODER_RESOLUTION
        # )
        # self.rightEncoder.setDistancePerPulse(
        #     2 * math.pi * self.WHEEL_RADIUS / self.ENCODER_RESOLUTION
        # )

        # self.leftEncoder.reset()
        # self.rightEncoder.reset()

        # self.odometry = DifferentialDriveOdometry(
        #     self.gyro.getRotation2d(),
        #     self.leftEncoder.getDistance(),
        #     self.rightEncoder.getDistance(),
        # )


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
        # movement using left and right joysticks
        self.drive.tankDrive(self.controller.getLeftY(), self.controller.getRightY())

        # # Ball Launcher (open loop)
        # if self.controller.getAButton(): # corresponds to BButton because its Logitech Dual Action to XboxController
        #     self.shooter.set(.2)
        # elif self.controller.getBButton(): # corresponds to XButton because its Logitech Dual Action to XboxController
        #     self.shooter.set(.4)
        # elif self.controller.getYButton():
        #     self.shooter.set(.6)
        # elif self.controller.getXButton(): # corresponds to AButton because its Logitech Dual Action to XboxController
        #     self.shooter.set(.8)
        # elif self.controller.getRightBumper() or self.controller.getLeftBumper():
        #     self.shooter.set(1)
        # else:
        #     self.shooter.set(0)

        # read PID coefficients from SmartDashboard
        p = SmartDashboard.getNumber("P Gain", 0)
        i = SmartDashboard.getNumber("I Gain", 0)
        d = SmartDashboard.getNumber("D Gain", 0)
        iz = SmartDashboard.getNumber("I Zone", 0)
        ff = SmartDashboard.getNumber("Feed Forward", 0)
        max = SmartDashboard.getNumber("Max Output", 0)
        min = SmartDashboard.getNumber("Min Output", 0)

        # if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != self.kP)):
            self.shooter_pidController.setP(p)
            self.kP = p
        if((i != self.kI)):
            self.shooter_pidController.setI(i)
            self.kI = i
        if((d != self.kD)):
            self.shooter_pidController.setD(d)
            self.kD = d
        if((iz != self.kIz)):
            self.shooter_pidController.setIZone(iz)
            self.kIz = iz
        if((ff != self.kFF)):
            self.shooter_pidController.setFF(ff)
            self.kFF = ff
        if((max != self.kMaxOutput) or (min != self.kMinOutput)):
            self.shooter_pidController.setOutputRange(min, max)
            self.kMinOutput = min
            self.kMaxOutput = max; 


    # SmartDashboard telemetry
        # SmartDashboard.putBoolean("AButton Status", self.controller.getAButton())
        # SmartDashboard.putBoolean("BButton Status", self.controller.getBButton())
        # SmartDashboard.putBoolean("YButton Status", self.controller.getYButton())
        # SmartDashboard.putBoolean("XButton Status", self.controller.getXButton())
        # SmartDashboard.putBoolean("Right Bumber Status", self.controller.getRightBumper())
        SmartDashboard.putNumber("TalonSRX front left motor power", self.frontleft_motor.get())
        SmartDashboard.putNumber("TalonSRX back left motor power", self.backleft_motor.get())
        SmartDashboard.putNumber("TalonSRX front right motor power", self.frontright_motor.get())
        SmartDashboard.putNumber("TalonSRX back right motor power", self.backright_motor.get())
        SmartDashboard.putBoolean("Robot_status_is_on", True)
    def teleopExit(self):
        SmartDashboard.putBoolean("Robot_status_is_on", False)

    def _simulationInit(self): pass
    def _simulationPeriodic(self): pass

    def disabledInit(self): pass
    def disabledPeriodic(self): pass
    def disabledExit(self): pass

if __name__ == '__main__':
    run(Robot)