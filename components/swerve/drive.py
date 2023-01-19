from components.swerve.module import SwerveModule
from components.swerve.module import SwerveModuleConfig

from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds
from wpimath.geometry import Rotation2d

from wpimath import applyDeadband

from wpilib import SmartDashboard as sd

import math

def _sign(n: float) -> int:
    if n == 0:
        return 0
    return round(n / abs(n))

def _cartesian_to_polar(point: tuple[float, float]) -> tuple[float, float]:
    x, y = point
    r = math.sqrt(x**2 + y**2)
    if x == 0:
        theta = math.atan(math.inf)
    else:
        theta = math.atan(y/x) * 180/math.pi
    if x < 0:
        theta += 180
    elif y < 0:
        theta += 360
    return r, theta

def _polar_to_cartesian(point: tuple[float, float]) -> tuple[float, float]:
    r, theta = point
    theta *= math.pi/180
    x = r * math.cos(theta)
    y = r * math.sin(theta)
    return x, y
    

class SwerveDrive:
    front_left: SwerveModule
    front_right: SwerveModule
    rear_left: SwerveModule
    rear_right: SwerveModule

    kinematics: SwerveDrive4Kinematics

    previous_speed: tuple[float, float] # polar


    def __init__(self, front_left_cfg: SwerveModuleConfig,
                       front_right_cfg: SwerveModuleConfig,
                       rear_left_cfg: SwerveModuleConfig,
                       rear_right_cfg: SwerveModuleConfig,
                       deadband: float = 0):
        self.front_left = SwerveModule(front_left_cfg)
        self.front_right = SwerveModule(front_right_cfg)
        self.rear_left = SwerveModule(rear_left_cfg)
        self.rear_right = SwerveModule(rear_right_cfg)

        self.modules = [self.front_left, self.front_right, self.rear_left, self.rear_right]

        self.deadband = deadband
        self.previous_speed = (0, 0)

        self.kinematics = SwerveDrive4Kinematics(front_left_cfg.relative_position, 
                                                 front_right_cfg.relative_position, 
                                                 rear_left_cfg.relative_position, 
                                                 rear_right_cfg.relative_position)
    
    def drive(self, xSpeed: float, ySpeed: float, rot: float, current_angle: float = 0, ramp_rate: float = 1) -> None:
        """
        Moves the drivetrain according to the inputs from the cartesian inputs. This method is 
        designed to easily take inputs from a controller. As such, inputs are expected to be 
        bounded within the range [-1, 1].

        The expected movement is as follows:
        - +x: right, -x: left
        - +y: forward, -y: reverse
        - +rot: CW, -rot: CCW


        You can enable field-relative drive by supplying the `current_angle` argument. Most often 
        this will be a value supplied by a gyro. However, to use traditional "robot-relative" drive,
        either don't supply the argument, or constantly set it to 0. This is because a robot-relative
        drive is the same as a field-relative drive, but where the "gyro" input is always 0 ("forward").

        Inputs are trimmed by the deadband value passed to the SwerveDrive constructor.
        """
        xSpeed = applyDeadband(xSpeed, self.deadband)
        ySpeed = applyDeadband(ySpeed, self.deadband)
        rot = applyDeadband(rot, self.deadband)
        if xSpeed == 0 and ySpeed == 0 and rot == 0:
            for m in self.modules:
                m.speed = 0
            return

        # sd.putNumber("X original", xSpeed)
        # sd.putNumber("Y original", ySpeed)
        # # r, theta = _cartesian_to_polar((xSpeed, ySpeed))
        # # # if abs(theta - self.previous_speed[1]) > 90:
        # # #     # if the speed is suddenly reversed
        # # #     r = 0    
        # # # else:
        # # #     r = self.previous_speed[0] + ramp_rate * (r - self.previous_speed[0])
        # # xSpeed, ySpeed = _polar_to_cartesian((r, theta))
        # sd.putNumber("X post", xSpeed)
        # sd.putNumber("Y post", ySpeed)
        # # self.previous_speed = (r, theta)

        
        # A field-relative drive with a constant rotation of 0 is just be a "normal" drive, eliminating 
        # the need to have a gyro in SwerveDrive itself, instead reserving it for the subsystem level
        chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, -ySpeed, -rot, Rotation2d.fromDegrees(current_angle))

        module_states = self.kinematics.toSwerveModuleStates(chassis_speeds)
        module_states = self.kinematics.desaturateWheelSpeeds(module_states, 1)
        for i, s in enumerate(module_states):
            self.modules[i].set_state(s)

    def initialize(self) -> None:
        """
        Puts the swerve module wheels facing parallel to the orientation of the robot.

        Optimizes the module angles to minimize wheel movement. Cancelled by any other drive call.
        """
        for m in self.modules:
            m.angle = m.closer_angle(90, -90)
            m.speed = 0

    def brace(self) -> None:
        """
        Put the swerve module wheels into an "X" position at a full stop, so as to maximize resistance 
        to being moved involuntarily.

        Optimizes angle of the "X" relative to the module's previous angle. Cancelled by any other drive call.
        """
        self.front_left.angle = self.front_left.closer_angle(45, -135)
        self.front_left.speed = 0
        
        self.front_right.angle = self.front_right.closer_angle(-45, 135)
        self.front_right.speed = 0

        self.rear_left.angle = self.rear_left.closer_angle(-45, 145)
        self.rear_left.speed = 0

        self.rear_right.angle = self.rear_right.closer_angle(45, -135)
        self.rear_right.speed = 0

    def stopMotor(self) -> None:
        for m in self.modules:
            m.stopMotor()