from components.swerve.drive import SwerveDrive
from components.swerve.module import SwerveModuleConfig
from wpimath.geometry import Translation2d
from commands2 import SubsystemBase
from wpilib import SPI


from navx import AHRS 

class DriveTrain(SubsystemBase):

    _drive: SwerveDrive
    gyro: AHRS

    rotation_offset: float

    def __init__(self, rotation_offset: float = 0, deadband: float = 0):
        front_left = SwerveModuleConfig(4, 3, 10, Translation2d(0.339725, 0.288925), False, 8.14)
        front_right = SwerveModuleConfig(6, 5, 11, Translation2d(0.339725, -0.288925), True, 8.14)
        rear_left = SwerveModuleConfig(2, 1, 9, Translation2d(-0.339725, 0.288925), False, 8.14)
        rear_right = SwerveModuleConfig(8, 7, 12, Translation2d(-0.339725, -0.288925), True, 8.14)
        self._drive = SwerveDrive(front_left, front_right, rear_left, rear_right, deadband)

        self.gyro = AHRS(SPI.Port.kMXP)

        self.rotation_offset = rotation_offset

        super().__init__()

    def drive(self, x: float, y:float, rot: float, field_relative: bool = False):
        if field_relative:
            self._drive.drive(x, y, rot, self.gyro.getYaw() + self.rotation_offset)
        else:
            self._drive.drive(x, y, rot)

    def reset(self) -> None:
        self._drive.initialize()

    def brace(self) -> None:
        self._drive.brace()
    
    def angle(self) -> float:
        return self.gyro.getAngle()
        
    