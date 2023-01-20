import wpilib


from subsystems.drivetrain import DriveTrain

class SwerveBot(wpilib.TimedRobot):
    controller: wpilib.XboxController
    
    drive: DriveTrain

    def robotInit(self) -> None:
        self.drive = DriveTrain(deadband = 0.1)
        self.controller = wpilib.XboxController(0)
    
    def teleopPeriodic(self) -> None:
        if self.controller.getYButtonPressed():
            self.drive.reset()
        elif self.controller.getXButtonPressed():
            self.drive.brace()
        elif self.controller.getBButtonPressed():
            self.drive.drive(0, 0, 0.25)
        
        self.drive.drive(self.controller.getLeftX(), 
                         -self.controller.getLeftY(), 
                         self.controller.getRightX(), 
                         field_relative = True,
                         ramp_rate = 0.01)

if __name__ == "__main__":
    wpilib.run(SwerveBot)