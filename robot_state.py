import math

from ntcore import NetworkTableInstance
from pathplannerlib.logging import PathPlannerLogging
from phoenix6 import swerve, utils
from wpilib import DataLogManager, DriverStation, Field2d, SmartDashboard, Mechanism2d, Color8Bit
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState

from lib.limelight import LimelightHelpers
from subsystems.elevator import ElevatorSubsystem
from subsystems.pivot import PivotSubsystem
from subsystems.swerve import SwerveSubsystem


class RobotState:

    def __init__(self, drivetrain: SwerveSubsystem, pivot: PivotSubsystem, elevator: ElevatorSubsystem):
        self._swerve = drivetrain
        self._pivot = pivot
        self._elevator = elevator

        DriverStation.startDataLog(DataLogManager.getLog())

        self._field = Field2d()
        SmartDashboard.putData("Field", self._field)
        self._field.setRobotPose(Pose2d())

        # Robot speeds for general checking
        self._table = NetworkTableInstance.getDefault().getTable("Telemetry")
        self._current_pose = self._table.getStructTopic("current_pose", Pose2d).publish()
        self._chassis_speeds = self._table.getStructTopic("chassis_speeds", ChassisSpeeds).publish()
        self._odom_freq = self._table.getDoubleTopic("odometry_frequency").publish()
        self._teleop_speed = self._table.getDoubleTopic("current_speed").publish()

        # Additional swerve info
        self._module_states = self._table.getStructArrayTopic("module_states", SwerveModuleState).publish()
        self._module_targets = self._table.getStructArrayTopic("module_targets", SwerveModuleState).publish()

        # Swerve Data
        self._swerve_data = self._table.getSubTable("Swerve Data")
        self._swerve_data.getEntry(".type").setString("SwerveDrive")  # Tells Elastic what widget this is

        PathPlannerLogging.setLogTargetPoseCallback(lambda pose: self._field.getObject("targetPose").setPose(pose))
        PathPlannerLogging.setLogActivePathCallback(lambda poses: self._field.getObject("activePath").setPoses(poses[::3]))

        if utils.is_simulation():
            self._superstructure_mechanism = Mechanism2d(0.5334, 2.286, Color8Bit("#000058"))
            self._root = self._superstructure_mechanism.getRoot("Root", 0.5334 / 2, 0.125)

            self._elevator_mech = self._root.appendLigament("Elevator", 0.2794, 90, 5, Color8Bit("#FFFFFF"))
            self._pivot_mech = self._elevator_mech.appendLigament("Pivot", 0.635, 0, 4, Color8Bit("#FEFEFE"))

            SmartDashboard.putData("Superstructure Mechanism", self._superstructure_mechanism)

    def log_swerve_state(self, state: swerve.SwerveDrivetrain.SwerveDriveState) -> None:
        """
        Logs desired info with the given swerve state. Called by the
        Phoenix 6 drivetrain method every time the odometry thread is
        updated.
        """

        self._field.setRobotPose(state.pose)
        self._current_pose.set(state.pose)

        self._odom_freq.set(1.0 / state.odometry_period)

        self._module_states.set(state.module_states)
        self._module_targets.set(state.module_targets)
        self._chassis_speeds.set(state.speeds)

        self._teleop_speed.set(abs(math.sqrt(state.speeds.vx ** 2 + state.speeds.vy ** 2)))

        self._swerve_data.getEntry("Front Left Angle").setDouble(state.module_states[0].angle.radians())
        self._swerve_data.getEntry("Front Left Velocity").setDouble(state.module_states[0].speed)
        self._swerve_data.getEntry("Front Right Angle").setDouble(state.module_states[1].angle.radians())
        self._swerve_data.getEntry("Front Right Velocity").setDouble(state.module_states[1].speed)
        self._swerve_data.getEntry("Back Left Angle").setDouble(state.module_states[2].angle.radians())
        self._swerve_data.getEntry("Back Left Velocity").setDouble(state.module_states[2].speed)
        self._swerve_data.getEntry("Back Right Angle").setDouble(state.module_states[3].angle.radians())
        self._swerve_data.getEntry("Back Right Velocity").setDouble(state.module_states[3].speed)
        self._swerve_data.getEntry("Robot Angle").setDouble((self._swerve.get_operator_forward_direction() + state.pose.rotation()).radians())

        NetworkTableInstance.getDefault().flush()

    def update_mechanisms(self) -> None:
        self._elevator_mech.setLength(self._elevator.get_height())
        self._pivot_mech.setAngle(self._pivot.get_angle())

    def get_current_pose(self) -> Pose2d:
        """Returns the current pose of the robot on the field (blue-side origin)."""
        return self._swerve.get_state().pose
    
    def add_vision_measurements(self, limelight_name: str, standard_deviation: tuple[float, float, float] = (0.7, 0.7, 9999999)) -> None:
        """
        Adds MegaTag2 Pose Estimates from the given limelight.

        Modified from Limelight's MegaTag 2 documentation, which can be found at
        https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2

        :param limelight_name: The name of the Limelight
        :type limelight_name: str
        :param standard_deviation: The standard deviations of the pose estimate. Lower values tell the pose estimator to trust the pose estimate more
        :type standard_deviation: tuple[float, float, float]
        :rtype: None
        """
        LimelightHelpers.set_robot_orientation(
            limelight_name,
            self._swerve.pigeon2.get_yaw().value,
            self._swerve.pigeon2.get_angular_velocity_z_world().value,
            self._swerve.pigeon2.get_pitch().value,
            self._swerve.pigeon2.get_angular_velocity_y_world().value,
            self._swerve.pigeon2.get_roll().value,
            self._swerve.pigeon2.get_angular_velocity_x_world().value
        )

        mega_tag2 = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(limelight_name)
        if abs(self._swerve.pigeon2.get_angular_velocity_z_world().value) <= 720 and mega_tag2.tag_count > 0:
            self._swerve.set_vision_measurement_std_devs(standard_deviation)
            self._swerve.add_vision_measurement(mega_tag2.pose, utils.fpga_to_current_time(mega_tag2.timestamp_seconds))

    def should_pivot_move(self) -> bool:
        return self._elevator.is_at_setpoint()

    def get_latency_compensated_pose(self, dt: float) -> Pose2d:
        """Returns the current pose of the robot on the field (blue-side origin),
        compensated for latency.

        :param dt: The amount of time in seconds since the last
            update.
        :type dt: float
        :return: The current pose of the robot on the field with
            latency compensation.
        :rtype: Pose2d
        """
        state = self._swerve.get_state()
        speeds = state.speeds
        pose = state.pose

        return Pose2d(
            pose.X() + speeds.vx * dt,
            pose.Y() + speeds.vy * dt,
            pose.rotation() + speeds.omega * dt
            )