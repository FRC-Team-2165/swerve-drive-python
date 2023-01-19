import math

import wpimath.kinematics as kinematics
from wpimath.geometry import Rotation2d

import ctre

from components.swerve.falcon_helper import *

SWERVE_WHEEL_RADIUS = 0.0508 # meters
SWERVE_WHEEL_CIRCUMFERENCE = 2 * math.pi * SWERVE_WHEEL_RADIUS 

from dataclasses import dataclass
from wpimath.geometry import Translation2d

@dataclass
class SwerveModuleConfig:
    drive_motor_id: int
    "The CAN id of the drive motor"
    turn_motor_id: int
    "The CAN id of the turn motor"
    turn_encoder_id: int
    "The CAN id of the CANCoder"
    relative_position: Translation2d # offset from the center of the robot
    "The position of the module, relative to the center of the robot."
    inverted: bool # Should the drive motor be inverted
    "The inversion of the drive motor. Turn motor can't be inverted."
    gear_ratio: float # The gear ratio of the drive wheel, in falcon rotations per wheel rotation
    """
    The gear ratio of the drive wheel, in Falcon rotations per wheel rotation.I

    If you only use the raw input methods, and not the speed-based ones (names containing "mps"),
    this value doesn't need to be set.
    """

class SwerveModule:
    """
    Represents a serve drive module based on Falcon 500 motors and the CTRE CANCoder magnetic encoder.
    Specifically designed for the MK4i swerve module kits from Swerve Drive Specialties.

    All angle control in these modules assumes that you have calibrated your CANCoders to be at 0 when
    the wheels are well-aligned to go forward. 

    Assumptions:
    - CANCoder
        - measures the angle range from [-180, 180)
        - has been offset so that the 0 position refers to when the wheel facing a quarter rotation right
        - clockwise rotations increase angle, counterclockwise decrease
        - relative value set to absolute position on boot
    - Turning Motor
        - PID is controlled by a CANCoder using the "remote sensor" setup
        - calibrated to produce a clockwise motion given a positive value, and a counterclockwise given a negative
        - set to "Brake" for neutral input (this is optional for drive motor, but recommended).
    - PIDF values have been set in the motors themselves
    """
    drive_motor: FalconMotor
    turn_motor: FalconMotor
    turn_encoder: ctre.WPI_CANCoder

    gear_ratio: float


    def __init__(self, config: SwerveModuleConfig):
        """
        Creates the swerve module according the given configuration. See SwerveModuleConfig for 
        more information on configuration options.
        """
        self.drive_motor = FalconMotor(config.drive_motor_id)
        self.turn_motor = FalconMotor(config.turn_motor_id)
        self.turn_encoder = ctre.WPI_CANCoder(config.turn_encoder_id)

        self.drive_motor.setInverted(config.inverted)

        self.gear_ratio = config.gear_ratio
        
    
    @property
    def inverted(self) -> bool:
        """
        Returns whether the drive motor is inverted. 
        """
        return self.drive_motor.getInverted()
    @inverted.setter
    def inverted(self, inverted: bool) -> None:
        """
        Sets whether the drive motor is inverted.

        This is initially set during configuration, and shouldn't need to be changed beyond that. 
        However, this is here if you have the use case.
        """
        self.drive_motor.setInverted(inverted) 

    def set_state(self, state: kinematics.SwerveModuleState) -> None:
        """
        Sets the state of the module, with speed being from [-1, 1]. Runs optimization to minimize heading change.
        """
        state = kinematics.SwerveModuleState.optimize(state, Rotation2d.fromDegrees(self.angle))
        self.angle = state.angle.degrees()
        self.speed = state.speed

    def set_state_mps(self, state: kinematics.SwerveModuleState) -> None:
        """
        Sets the state of the module, with speed in m/s. Runs optimization to minimize heading change.
        """
        state = kinematics.SwerveModuleState.optimize(state, Rotation2d.fromDegrees(self.angle))
        self.angle = state.angle.degrees()
        self.speed_mps = state.speed

    def get_state(self) -> kinematics.SwerveModuleState:
        """
        Returns a representation of the current state of the module.
        """
        return kinematics.SwerveModuleState(self.speed_mps, self.angle)

    @property
    def angle(self) -> float:
        """Returns the current module angle in degrees"""
        return self.turn_encoder.getAbsolutePosition()

    @angle.setter
    def angle(self, angle: float) -> None:
        """Sets the current angle in degrees. Values outside the range [-180, 180) will be normalized in that range."""
        angle = (angle + 180) % 360 - 180
        self.turn_motor.set(ctre.ControlMode.Position, degrees_to_CANCoder(angle))
        # if (abs(angle - self.angle) <= delta):
        #     self.turn_motor.set(0)
        # if angle > self.angle:
        #     self.turn_motor.set(-0.8)
        # elif angle < self.angle:
        #     self.turn_motor.set(0.8)
            
    @property
    def speed(self) -> float:
        """Returns a value in the range [-1, 1] representing the drive motor's approximate speed."""
        return self.drive_motor.get()
    
    @speed.setter
    def speed(self, speed: float) -> None:
        """Sets the speed of the drive motor in the range [-1, 1]. Values outside the range will be clipped."""
        if(abs(speed) > 1):
            speed /= abs(speed) # cap, while retaining sign
        self.drive_motor.set(speed)

    def stopMotor(self) -> None:
        self.drive_motor.stopMotor()
        self.turn_motor.stopMotor()
    
    @property
    def speed_mps(self) -> float:
        """Returns the drive motor's approximate speed, in meters per second."""
        return falcon_to_mps(self.speed, SWERVE_WHEEL_CIRCUMFERENCE, self.gear_ratio)

    @speed_mps.setter
    def speed_mps(self, speed: float) -> None:
        """Sets the drive motor's speed, in meters per second"""
        self.speed = mps_to_falcon(speed, SWERVE_WHEEL_CIRCUMFERENCE, self.gear_ratio)

    def closer_angle(self, angle1: float, angle2: float) -> float:
        """
        Returns the angle that is closer to the module's current angle.

        This is most useful for angle optimization when the drive motor is not being used,
        for example to brace the robot by setting the wheels in a certain position.
        """
        angle = self.angle
        if abs(self.angle - angle1) < abs(self.angle - angle2):
            return angle1
        else:
            return angle2

