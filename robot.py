import wpilib

sd = wpilib.SmartDashboard

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
        elif self.controller.getRightBumperPressed():
            self.drive.gyro.reset()


        self.drive.drive(self.controller.getLeftX(), 
                         -self.controller.getLeftY(), 
                         self.controller.getRightX(), 
                         field_relative = True,
                         square_inputs = False)

        pos = self.drive._drive.position()
        sd.putNumber("Robot X", pos.X())
        sd.putNumber("Robot Y", pos.Y())
        sd.putNumber("wheel speed", self.drive._drive.modules[0].speed_mps)
        raw_pos = self.drive._drive._position.to_cartesian()
        sd.putNumber("Robot Raw X", raw_pos.x)
        sd.putNumber("Robot Raw Y", raw_pos.y)
                                

if __name__ == "__main__":
    wpilib.run(SwerveBot)
