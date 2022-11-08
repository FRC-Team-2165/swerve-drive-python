import wpilib


from subsystems.drivetrain import DriveTrain

class SwerveBot(wpilib.TimedRobot):
    controller: wpilib.XboxController
    
    drive: DriveTrain

    def robotInit(self) -> None:
        self.drive = DriveTrain(0)
        self.controller = wpilib.XboxController(0)
    
    def teleopPeriodic(self) -> None:
        self.drive.drive(self.controller.getLeftX(), -self.controller.getLeftY(), self.controller.getRightY(), False)