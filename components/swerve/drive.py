from components.swerve.module import SwerveModule, SWERVE_WHEEL_CIRCUMFERENCE
from components.swerve.module import SwerveModuleConfig

from typing import Tuple

from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds
from wpimath.geometry import Rotation2d

from wpimath import applyDeadband

from math import pi

from components.falcon_helper import mps_to_falcon

# from navx import AHRS

from wpilib import SmartDashboard

class SwerveDrive:
    MAX_SPEED = 3 # Meters per second

    front_left: SwerveModule
    front_right: SwerveModule
    rear_left: SwerveModule
    rear_right: SwerveModule

    # gyro: AHRS

    kinematics: SwerveDrive4Kinematics


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

        # self.gyro = AHRS(gyro_port)
        # self.gyro.reset()

        self.kinematics = SwerveDrive4Kinematics(front_left_cfg.relative_position, 
                                                 front_right_cfg.relative_position, 
                                                 rear_left_cfg.relative_position, 
                                                 rear_right_cfg.relative_position)
    
    # def drive(self, xSpeed: float, ySpeed: float, rot: float, field_relative: bool = False) -> None:
    #     if field_relative:
    #         chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, self.gyro.getAngle())
    #     else:
    #         chassis_speeds = ChassisSpeeds(xSpeed, ySpeed, rot)
    def drive(self, xSpeed: float, ySpeed: float, rot: float, current_angle: float = 0) -> None:
        xSpeed = applyDeadband(xSpeed, self.deadband)
        ySpeed = applyDeadband(ySpeed, self.deadband)
        rot = applyDeadband(rot, self.deadband)
        SmartDashboard.putNumber("Drive Input: X", xSpeed)
        SmartDashboard.putNumber("Drive Input: Y", ySpeed)
        SmartDashboard.putNumber("Drive Input: rotation", rot)
        if xSpeed == 0 and ySpeed == 0 and rot == 0:
            return
        # Theoretically, a field-relative drive with a constant rotation of 0 should just be a "normal" drive,
        # eliminating the need to have a gyro in SwerveDrive itself, instead reserving it for the subsystem level
        chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, -xSpeed, rot, Rotation2d.fromDegrees(current_angle))

        module_states = self.kinematics.toSwerveModuleStates(chassis_speeds)
        m = module_states[0]
        SmartDashboard.putNumber("Output speed(raw)", m.speed)
        SmartDashboard.putNumber("Output rotations(raw)", mps_to_falcon(m.speed, SWERVE_WHEEL_CIRCUMFERENCE, 8.14))
        module_states = self.kinematics.desaturateWheelSpeeds(module_states, 1)
        m = module_states[0]
        SmartDashboard.putNumber("Output speed", m.speed)
        SmartDashboard.putNumber("Output rotations", mps_to_falcon(m.speed, SWERVE_WHEEL_CIRCUMFERENCE, 8.14))
        SmartDashboard.putNumber("Module angle", self.modules[0].angle)
        for i, s in enumerate(module_states):
            self.modules[i].set_state(s)
        # self.front_left.set_state(module_states[0])
        # self.front_right.set_state(module_states[1])
        # self.rear_left.set_state(module_states[2])
        # self.rear_right.set_state(module_states[3])

    def initialize(self) -> None:
        for m in self.modules:
            m.angle = 0
            m.speed = 0

    def brace(self) -> None:
        """
        Put the swerve module wheels into an "X" position at a full stop, so as to maximize resistance to being moved involuntarily.

        Works independently of prior wheel angle . Cancelled by calls to `drive`.
        """
        self.front_left.angle = -45
        self.front_left.speed = 0
        
        self.front_right.angle = 45
        self.front_right.speed = 0

        self.rear_left.angle = 45
        self.rear_left.speed = 0

        self.rear_right.angle = -45
        self.rear_right.speed = 0
