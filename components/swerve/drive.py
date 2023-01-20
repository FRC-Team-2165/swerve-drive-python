from components.swerve.module import SwerveModule, SwerveModuleConfig
from components.swerve.polar import Polar


from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds
from wpimath.geometry import Rotation2d

from wpimath import applyDeadband

from wpilib import SmartDashboard as sd

import math

def _sign(n: float) -> int:
    if n == 0:
        return 0
    return round(n / abs(n))


def _cartesian_to_polar(point: tuple[float, float]) -> Polar:
    x, y = point
    r = math.sqrt(x**2 + y**2)
    if x == 0:
        if y > 0:
            return Polar(y, 90)
        elif y < 0:
            return Polar(-y, 270)
        else:
            return Polar(0, 0)
    else:
        theta = math.atan(y/x) * 180/math.pi
    if x < 0:
        theta += 180
    elif y < 0:
        theta += 360
    return Polar(r, theta)

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

    previous_speed: float


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

        # self.kinematics = SwerveDrive4Kinematics(front_left_cfg.relative_position, 
        #                                          front_right_cfg.relative_position, 
        #                                          rear_left_cfg.relative_position, 
        #                                          rear_right_cfg.relative_position)
    
    def drive(self, xSpeed: float, ySpeed: float, rot: float, current_angle: float = 0, square_inputs: bool = False, ramp_rate: float = 1) -> None:
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
        sd.putNumber("rot (raw)", rot)

        if square_inputs:
            xSpeed *= abs(xSpeed)
            ySpeed *= abs(ySpeed)
            rot *= abs(rot)

        xSpeed = applyDeadband(xSpeed, self.deadband)
        ySpeed = applyDeadband(ySpeed, self.deadband)
        rot = applyDeadband(rot, self.deadband)

        sd.putNumber("rot (post-deadband)", rot)
        # if xSpeed == 0 and ySpeed == 0 and rot == 0:
        #     for m in self.modules:
        #         m.speed = 0
        #     return


        
        # A field-relative drive with a constant rotation of 0 is just be a "normal" drive, eliminating 
        # the need to have a gyro in SwerveDrive itself, instead reserving it for the subsystem level
        # chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, -ySpeed, -rot, Rotation2d.fromDegrees(current_angle))

        # module_states = self.kinematics.toSwerveModuleStates(chassis_speeds)

        # Convert cartesian vector input to polar vector. Makes all of the math *much* simpler.
        target_vector = _cartesian_to_polar((xSpeed, -ySpeed))
        sd.putNumber("Target angle", target_vector.theta)
        sd.putNumber("Target Speed", target_vector.magnitude)
        # adjust target_vector angle by the current heading
        sd.putNumber("Current angle", current_angle)
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
            combined_magnitude = abs(rot) + target_vector.magnitude
            if combined_magnitude == 0:
                rotation_weight = 0
                target_weight = 0
            else:
                rotation_weight = abs(rot) / combined_magnitude
                target_weight = target_vector.magnitude / combined_magnitude

            module_angle = target_vector.theta * target_weight + rotation_angle * rotation_weight
            module_speed = target_vector.magnitude * target_weight + rotation_speed * rotation_weight
            module_states.append(Polar(module_speed, module_angle))


        # # Convert SwerveModuleState to something useful
        # module_states = [(m.angle, m.speed) for m in module_states]

        # Desaturate module speeds. May not be necessary?
        top_speed = max(m.magnitude for m in module_states)
        if top_speed > 1:
            module_states = [Polar(m.magnitude/top_speed, m.theta) for m in module_states]
 
        for i, s in enumerate(module_states):
            sd.putNumber("Module {} speed".format(i), s.magnitude)
            sd.putNumber("Module {} angle".format(i), s.theta)
            if s.magnitude == 0:
                self.modules[i].speed = 0
            else:
                self.modules[i].set_state(s)

    def initialize(self) -> None:
        """
        Puts the swerve module wheels facing parallel to the orientation of the robot.

        Optimizes the module angles to minimize wheel movement. Cancelled by any other drive call.
        """
        for m in self.modules:
            m.set_state(Polar(0, 90))
            # m.angle = m._closer_angle(90, -90)
            # m.speed = 0

    def brace(self) -> None:
        """
        Put the swerve module wheels into an "X" position at a full stop, so as to maximize resistance 
        to being moved involuntarily.

        Optimizes angle of the "X" relative to the module's previous angle. Cancelled by any other drive call.
        """
        self.front_left.set_state(Polar(0, 45))
        # self.front_left.angle = self.front_left._closer_angle(45, -135)
        # self.front_left.speed = 0
        self.front_right.set_state(Polar(0, -45))
        # self.front_right.angle = self.front_right._closer_angle(-45, 135)
        # self.front_right.speed = 0
        self.rear_left.set_state(Polar(0, -45))
        # self.rear_left.angle = self.rear_left._closer_angle(-45, 135)
        # self.rear_left.speed = 0
        self.rear_right.set_state(Polar(0, 45))
        # self.rear_right.angle = self.rear_right._closer_angle(45, -135)
        # self.rear_right.speed = 0

    def stopMotor(self) -> None:
        for m in self.modules:
            m.stopMotor()