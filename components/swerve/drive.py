from components.swerve.module import SwerveModule, SwerveModuleConfig
from components.swerve.vector import Polar, Cartesian

from wpimath import applyDeadband
from wpimath.geometry import Translation2d, Pose2d

from wpilib import SmartDashboard as sd
from wpilib.drive import RobotDriveBase

from copy import copy

class SwerveDrive(RobotDriveBase):
    front_left: SwerveModule
    front_right: SwerveModule
    rear_left: SwerveModule
    rear_right: SwerveModule

    _position: Polar


    def __init__(self, front_left_cfg: SwerveModuleConfig,
                       front_right_cfg: SwerveModuleConfig,
                       rear_left_cfg: SwerveModuleConfig,
                       rear_right_cfg: SwerveModuleConfig,
                       deadband: float = 0):

        super().__init__()
        self.front_left = SwerveModule(front_left_cfg)
        self.front_right = SwerveModule(front_right_cfg)
        self.rear_left = SwerveModule(rear_left_cfg)
        self.rear_right = SwerveModule(rear_right_cfg)

        self.modules = [self.front_left, self.front_right, self.rear_left, self.rear_right]

        self.deadband = deadband

        self._position = Polar(0, 0)

        

    def setDeadband(self, deadband: float) -> None:
        self.deadband = deadband

    def getDescription(self) -> str:
        return "SwerveDrive"
    
    def drive(self, xSpeed: float, ySpeed: float, rot: float, current_angle: float = 0, square_inputs: bool = False) -> None:
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
        if square_inputs:
            xSpeed *= abs(xSpeed)
            ySpeed *= abs(ySpeed)
            rot *= abs(rot)

        xSpeed = applyDeadband(xSpeed, self.deadband)
        ySpeed = applyDeadband(ySpeed, self.deadband)
        rot = applyDeadband(rot, self.deadband)


        # Convert cartesian vector input to polar vector. Makes all of the math *much* simpler.
        target_vector = Cartesian(xSpeed, -ySpeed).to_polar()
        target_vector.theta -= current_angle

        max_module_distance = max(m.offset_from_center() for m in self.modules)

        module_states: list[Polar] = []
        # Combine target vector with rotation state for each module.
        # Final vector is a weighted average of rotation and target vectors, based on proportion of combined magnitude.
        for m in self.modules:
            rotation_angle = m.rotation_angle()
            if rot > 0: 
                rotation_angle = (rotation_angle + 180) % 360
            rotation_speed = abs(rot) * (m.offset_from_center() / max_module_distance)
            rotation = Polar(rotation_speed, rotation_angle)

            module_states.append(target_vector + rotation)

        # Desaturate module speeds. Very necessary.
        top_speed = max(m.magnitude for m in module_states)
        if top_speed > 1:
            for m in module_states:
                m.magnitude /= top_speed


        for i, s in enumerate(module_states):
            if s.magnitude == 0:
                # only stop the drive motors, but don't reset the angle
                self.modules[i].speed = 0
            else:
                self.modules[i].set_state(s)
        self.feed()

        net_vector = sum((m.get_state_mps() for m in self.modules), start=Polar(0, 0))
        net_vector.theta += current_angle
        net_vector.magnitude *= 0.02 / len(self.modules)
        self._position += net_vector
        

    def initialize(self) -> None:
        """
        Puts the swerve module wheels facing parallel to the orientation of the robot.

        Optimizes the module angles to minimize wheel movement. Cancelled by any other drive call.
        """
        for m in self.modules:
            m.set_state(Polar(0, 90))
        self.feed()

    def brace(self) -> None:
        """
        Put the swerve module wheels into an "X" position at a full stop, so as to maximize resistance 
        to being moved involuntarily.

        Optimizes angle of the "X" relative to the module's previous angle. Cancelled by any other drive call.
        """
        self.front_left.set_state(Polar(0, 45))
        self.front_right.set_state(Polar(0, -45))
        self.rear_left.set_state(Polar(0, -45))
        self.rear_right.set_state(Polar(0, 45))
        self.feed()

    def stopMotor(self) -> None:
        """
        Disables the drive until `drive` is called again.
        """
        for m in self.modules:
            m.stopMotor()
        self.feed()
    
    def position(self) -> Translation2d:
        # TODO: Make more implementation agnostic way of solving. Currently relies on relative_position, 
        # which technically shouldn't be public
        # Also maybe can't handle rotation at present. This may be a problem that cancels itself out though.

        trans = self._position.to_translation2d()
        return Translation2d(-trans.X(), trans.Y())

    def reset_position(self) -> None:
        """
        Resets the tracked position of the drive. Does not affect the state of the drive, only the odometry.
        """
        self._position = Translation2d()
    
