import commands2
import commands2.button
from commands2 import cmd
from commands2.sysid import SysIdRoutine
from pathplannerlib.auto import AutoBuilder, NamedCommands
from pathplannerlib.path import PathConstraints, PathPlannerPath
from phoenix6 import SignalLogger, swerve
from wpilib import DriverStation, SmartDashboard
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians

from generated.tuner_constants import TunerConstants
from robot_state import RobotState
from subsystems.climber import ClimberSubsystem
from subsystems.elevator import ElevatorSubsystem
from subsystems.funnel import FunnelSubsystem
from subsystems.intake import IntakeSubsystem
from subsystems.pivot import PivotSubsystem
from subsystems.superstructure import Superstructure


class RobotContainer:

    def __init__(self) -> None:
        self._max_speed = (
            TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            1
        )  # 3/4 of a rotation per second max angular velocity

        self._driver_controller = commands2.button.CommandXboxController(0)
        self._function_controller = commands2.button.CommandXboxController(1)
        self.path_constraints = PathConstraints(1, 1, 1, 1, unlimited=False)
        self.trigger_margin = .75

        self.drivetrain = TunerConstants.create_drivetrain()

        self.climber = ClimberSubsystem()
        self.pivot = PivotSubsystem()
        self.intake = IntakeSubsystem()
        self.elevator = ElevatorSubsystem()
        self.funnel = FunnelSubsystem()

        self.robot_state = RobotState(self.drivetrain, self.pivot, self.elevator)
        self.superstructure = Superstructure(self.drivetrain, self.pivot, self.elevator, self.funnel, self.robot_state)

        # PathPlanner Commands

        NamedCommands.registerCommand("Ground Intaking", self.superstructure.set_goal_command(self.superstructure.Goal.GROUND_INTAKE))
        NamedCommands.registerCommand("Funnel Intaking", self.superstructure.set_goal_command(self.superstructure.Goal.FUNNEL_INTAKE))

        NamedCommands.registerCommand("L1 Coral Scoring", self.superstructure.set_goal_command(self.superstructure.Goal.L1_SCORING))
        NamedCommands.registerCommand("L2 Coral Scoring", self.superstructure.set_goal_command(self.superstructure.Goal.L2_SCORING))
        NamedCommands.registerCommand("L3 Coral Scoring", self.superstructure.set_goal_command(self.superstructure.Goal.L3_SCORING))
        NamedCommands.registerCommand("L4 Coral Scoring", self.superstructure.set_goal_command(self.superstructure.Goal.L4_SCORING))

        NamedCommands.registerCommand("Algae Proccessor Scoring", self.superstructure.set_goal_command(self.superstructure.Goal.ALGAE_SCORING_PROCESSOR))
        NamedCommands.registerCommand("Algae Net Scoring", self.superstructure.set_goal_command(self.superstructure.Goal.ALGAE_SCORING_NET))

        NamedCommands.registerCommand("L2 Algae Intaking", self.superstructure.set_goal_command(self.superstructure.Goal.L2_ALGAE_INTAKE))
        NamedCommands.registerCommand("L3 Algae Intaking", self.superstructure.set_goal_command(self.superstructure.Goal.L3_ALGAE_INTAKE))

        NamedCommands.registerCommand("Intake Intake", self.intake.set_desired_state_command(IntakeSubsystem.SubsystemState.INTAKING))
        NamedCommands.registerCommand("Output Intake", self.intake.set_desired_state_command(IntakeSubsystem.SubsystemState.OUTPUTTING))
        NamedCommands.registerCommand("Stop Intake", self.intake.set_desired_state_command(IntakeSubsystem.SubsystemState.DEFAULT))


        # These are the paths that the robot can follow, which are preloaded so we reference them later and reduce lag.
        self.preloaded_paths = {
            "Coral A" : PathPlannerPath.fromPathFile("Coral A"),
            "Coral B" : PathPlannerPath.fromPathFile("Coral B"),
            "Coral C" : PathPlannerPath.fromPathFile("Coral C"),
            "Coral D" : PathPlannerPath.fromPathFile("Coral D"),
            "Coral E" : PathPlannerPath.fromPathFile("Coral E"),
            "Coral F" : PathPlannerPath.fromPathFile("Coral F"),
            "Coral G" : PathPlannerPath.fromPathFile("Coral G"),
            "Coral H" : PathPlannerPath.fromPathFile("Coral H"),
            "Coral I" : PathPlannerPath.fromPathFile("Coral I"),
            "Coral J" : PathPlannerPath.fromPathFile("Coral J"),
            "Coral K" : PathPlannerPath.fromPathFile("Coral K"),
            "Coral L" : PathPlannerPath.fromPathFile("Coral L"),
            "Coral Station 1" : PathPlannerPath.fromPathFile("Coral Station 1"),
            "Coral Station 2" : PathPlannerPath.fromPathFile("Coral Station 2"),
        } # Ends the dictionary

        # Setting up bindings for necessary control of the swerve drive platform
        self._field_centric = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.01)
            .with_rotational_deadband(
                self._max_angular_rate * 0.01
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._robot_centric = (
            swerve.requests.RobotCentric()
            .with_deadband(self._max_speed * 0.01)
            .with_rotational_deadband(
                self._max_angular_rate * 0.01
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

        # Path follower
        self._auto_chooser = AutoBuilder.buildAutoChooser("Auto Chooser")
        SmartDashboard.putData("Auto Mode", self._auto_chooser)

        # Configure the button bindings
        self.configure_button_bindings()

    def configure_button_bindings(self) -> None:
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(
                lambda: (
                    self._field_centric.with_velocity_x(
                        -self._driver_controller.getLeftY() * self._max_speed
                    )
                    .with_velocity_y(
                        -self._driver_controller.getLeftX() * self._max_speed
                    )
                    .with_rotational_rate(
                        -self._driver_controller.getRightX() * self._max_angular_rate
                    )
                )
            )
        )

        self._driver_controller.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        self._driver_controller.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(-self._driver_controller.getLeftY(), -self._driver_controller.getLeftX())
                )
            )
        )

        (self._driver_controller.back() & self._driver_controller.y()).onTrue(commands2.InstantCommand(lambda: SignalLogger.start())).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward).onlyIf(lambda: not DriverStation.isFMSAttached())
        )
        (self._driver_controller.back() & self._driver_controller.x()).onTrue(commands2.InstantCommand(lambda: SignalLogger.start())).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse).onlyIf(lambda: not DriverStation.isFMSAttached())
        )
        (self._driver_controller.start() & self._driver_controller.y()).onTrue(commands2.InstantCommand(lambda: SignalLogger.start())).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward).onlyIf(lambda: not DriverStation.isFMSAttached())
        )
        (self._driver_controller.start() & self._driver_controller.x()).onTrue(commands2.InstantCommand(lambda: SignalLogger.start())).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse).onlyIf(lambda: not DriverStation.isFMSAttached())
        )

        (commands2.button.Trigger(lambda: self._driver_controller.getLeftTriggerAxis() < self.trigger_margin) & commands2.button.Trigger(lambda: self._driver_controller.getRightTriggerAxis() < self.trigger_margin) & self._driver_controller.leftBumper()).whileTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        )

        #Left reef sides
        (commands2.button.Trigger(lambda: self._driver_controller.getLeftTriggerAxis() >= self.trigger_margin) & self._driver_controller.y()).whileTrue(
                    AutoBuilder.pathfindThenFollowPath(self.preloaded_paths["Coral A"], self.path_constraints)
                )
        (commands2.button.Trigger(lambda: self._driver_controller.getLeftTriggerAxis() >= self.trigger_margin) & self._driver_controller.x()).whileTrue(
                    AutoBuilder.pathfindThenFollowPath(self.preloaded_paths["Coral C"], self.path_constraints)
                )
        (commands2.button.Trigger(lambda: self._driver_controller.getLeftTriggerAxis() >= self.trigger_margin) & self._driver_controller.a()).whileTrue(
                    AutoBuilder.pathfindThenFollowPath(self.preloaded_paths["Coral E"], self.path_constraints)
                )
        (commands2.button.Trigger(lambda: self._driver_controller.getLeftTriggerAxis() >= self.trigger_margin) & self._driver_controller.b()).whileTrue(
                    AutoBuilder.pathfindThenFollowPath(self.preloaded_paths["Coral G"], self.path_constraints)
                )
        (commands2.button.Trigger(lambda: self._driver_controller.getLeftTriggerAxis() >= self.trigger_margin) & self._driver_controller.rightBumper()).whileTrue(
                    AutoBuilder.pathfindThenFollowPath(self.preloaded_paths["Coral I"], self.path_constraints)
                )
        (commands2.button.Trigger(lambda: self._driver_controller.getLeftTriggerAxis() >= self.trigger_margin) & self._driver_controller.leftBumper()).whileTrue(
                    AutoBuilder.pathfindThenFollowPath(self.preloaded_paths["Coral K"], self.path_constraints)
                )  

        #right reef sides
        (commands2.button.Trigger(lambda: self._driver_controller.getRightTriggerAxis() >= self.trigger_margin) & self._driver_controller.y()).whileTrue(
                    AutoBuilder.pathfindThenFollowPath(self.preloaded_paths["Coral B"], self.path_constraints)
                )
        (commands2.button.Trigger(lambda: self._driver_controller.getRightTriggerAxis() >= self.trigger_margin) & self._driver_controller.x()).whileTrue(
                    AutoBuilder.pathfindThenFollowPath(self.preloaded_paths["Coral D"], self.path_constraints)
                )
        (commands2.button.Trigger(lambda: self._driver_controller.getRightTriggerAxis() >= self.trigger_margin) & self._driver_controller.a()).whileTrue(
                    AutoBuilder.pathfindThenFollowPath(self.preloaded_paths["Coral F"], self.path_constraints)
                )
        (commands2.button.Trigger(lambda: self._driver_controller.getRightTriggerAxis() >= self.trigger_margin) & self._driver_controller.b()).whileTrue(
                    AutoBuilder.pathfindThenFollowPath(self.preloaded_paths["Coral H"], self.path_constraints)
                )
        (commands2.button.Trigger(lambda: self._driver_controller.getRightTriggerAxis() >= self.trigger_margin) & self._driver_controller.rightBumper()).whileTrue(
                    AutoBuilder.pathfindThenFollowPath(self.preloaded_paths["Coral J"], self.path_constraints)
                )
        (commands2.button.Trigger(lambda: self._driver_controller.getRightTriggerAxis() >= self.trigger_margin) & self._driver_controller.leftBumper()).whileTrue(
                    AutoBuilder.pathfindThenFollowPath(self.preloaded_paths["Coral L"], self.path_constraints)
                )
        
        #coral stations
        (commands2.button.Trigger(lambda: self._driver_controller.getRightTriggerAxis() >= self.trigger_margin) & commands2.button.Trigger(lambda: self._driver_controller.getLeftTriggerAxis() >= self.trigger_margin) & self._driver_controller.leftBumper()).whileTrue(
                    AutoBuilder.pathfindThenFollowPath(self.preloaded_paths["Coral Station 1"], self.path_constraints)
                )
        (commands2.button.Trigger(lambda: self._driver_controller.getRightTriggerAxis() >= self.trigger_margin) & commands2.button.Trigger(lambda: self._driver_controller.getLeftTriggerAxis() >= self.trigger_margin) & self._driver_controller.rightBumper()).whileTrue(
                    AutoBuilder.pathfindThenFollowPath(self.preloaded_paths["Coral Station 2"], self.path_constraints)
                )

        (commands2.button.Trigger(lambda: self._driver_controller.getLeftTriggerAxis() < self.trigger_margin) & commands2.button.Trigger(lambda: self._driver_controller.getRightTriggerAxis() < self.trigger_margin) & self._driver_controller.rightBumper()).whileTrue( #Only does this function if the triggers aren't pressed

            self.drivetrain.apply_request(
                lambda: (
                    self._robot_centric.with_velocity_x(
                        -self._driver_controller.getLeftY() * self._max_speed
                    )
                    .with_velocity_y(
                        -self._driver_controller.getLeftX() * self._max_speed
                    )
                    .with_rotational_rate(
                        -self._driver_controller.getRightX() * self._max_angular_rate
                    )
                )
            )
        )

        self._function_controller.y().onTrue(
            self.superstructure.set_goal_command(self.superstructure.Goal.L4_SCORING)
        )

        self._function_controller.x().onTrue(
            self.superstructure.set_goal_command(self.superstructure.Goal.L3_SCORING)
        )

        self._function_controller.b().onTrue(
            self.superstructure.set_goal_command(self.superstructure.Goal.L2_SCORING)
        )

        self._function_controller.a().onTrue(
            self.superstructure.set_goal_command(self.superstructure.Goal.L1_SCORING)
        )

        (self._function_controller.y() & self._function_controller.start()).onTrue(
            self.superstructure.set_goal_command(self.superstructure.Goal.ALGAE_SCORING_NET)
        )

        (self._function_controller.x() & self._function_controller.start()).onTrue(
            self.superstructure.set_goal_command(self.superstructure.Goal.L3_ALGAE_INTAKE)
        )

        (self._function_controller.b() & self._function_controller.start()).onTrue(
            self.superstructure.set_goal_command(self.superstructure.Goal.L2_ALGAE_INTAKE)
        )

        (self._function_controller.a() & self._function_controller.start()).onTrue(
            self.superstructure.set_goal_command(self.superstructure.Goal.ALGAE_SCORING_PROCESSOR)
        )

        self._function_controller.leftBumper().whileTrue(
            cmd.parallel(
                self.superstructure.set_goal_command(self.superstructure.Goal.FUNNEL_INTAKE),
                self.intake.set_desired_state_command(self.intake.SubsystemState.INTAKING)
            )
        )

        (self._function_controller.leftBumper() & self._function_controller.back()).whileTrue(
            cmd.parallel(
                self.superstructure.set_goal_command(self.superstructure.Goal.GROUND_INTAKE),
                self.intake.set_desired_state_command(self.intake.SubsystemState.INTAKING)
            )
        )

        self._function_controller.rightBumper().whileTrue(
            self.intake.set_desired_state_command(self.intake.SubsystemState.OUTPUTTING)
        ).onFalse(self.intake.set_desired_state_command(self.intake.SubsystemState.DEFAULT))

        (self._function_controller.leftStick() & self._function_controller.rightStick()).whileTrue(
            self.superstructure.set_goal_command(self.superstructure.Goal.DEFAULT)
        )

        self._function_controller.povLeft().whileTrue(
            self.climber.set_desired_state_command(self.climber.SubsystemState.CLIMB_POSITIVE)
        ).onFalse(
            self.climber.set_desired_state_command(self.climber.SubsystemState.STOP)
        )

        self._function_controller.povRight().whileTrue(
            self.climber.set_desired_state_command(self.climber.SubsystemState.CLIMB_NEGATIVE)
        ).onFalse(
            self.climber.set_desired_state_command(self.climber.SubsystemState.STOP)
        )

        (self._function_controller.povUp() & self._function_controller.y()).onTrue(commands2.InstantCommand(lambda: SignalLogger.start())).whileTrue(
            self.elevator.sys_id_dynamic(SysIdRoutine.Direction.kForward).onlyIf(lambda: not DriverStation.isFMSAttached())
        )
        (self._function_controller.povUp() & self._function_controller.a()).onTrue(commands2.InstantCommand(lambda: SignalLogger.start())).whileTrue(
            self.elevator.sys_id_dynamic(SysIdRoutine.Direction.kReverse).onlyIf(lambda: not DriverStation.isFMSAttached())
        )
        (self._function_controller.povDown() & self._function_controller.y()).onTrue(commands2.InstantCommand(lambda: SignalLogger.start())).whileTrue(
            self.elevator.sys_id_quasistatic(SysIdRoutine.Direction.kForward).onlyIf(lambda: not DriverStation.isFMSAttached())
        )
        (self._function_controller.povDown() & self._function_controller.a()).onTrue(commands2.InstantCommand(lambda: SignalLogger.start())).whileTrue(
            self.elevator.sys_id_quasistatic(SysIdRoutine.Direction.kReverse).onlyIf(lambda: not DriverStation.isFMSAttached())
        )

        (self._function_controller.povLeft() & self._function_controller.y()).onTrue(commands2.InstantCommand(lambda: SignalLogger.start())).whileTrue(
            self.pivot.sys_id_dynamic(SysIdRoutine.Direction.kForward).onlyIf(lambda: not DriverStation.isFMSAttached())
        )
        (self._function_controller.povLeft() & self._function_controller.a()).onTrue(commands2.InstantCommand(lambda: SignalLogger.start())).whileTrue(
            self.pivot.sys_id_dynamic(SysIdRoutine.Direction.kReverse).onlyIf(lambda: not DriverStation.isFMSAttached())
        )
        (self._function_controller.povRight() & self._function_controller.y()).onTrue(commands2.InstantCommand(lambda: SignalLogger.start())).whileTrue(
            self.pivot.sys_id_quasistatic(SysIdRoutine.Direction.kForward).onlyIf(lambda: not DriverStation.isFMSAttached())
        )
        (self._function_controller.povRight() & self._function_controller.a()).onTrue(commands2.InstantCommand(lambda: SignalLogger.start())).whileTrue(
            self.pivot.sys_id_quasistatic(SysIdRoutine.Direction.kReverse).onlyIf(lambda: not DriverStation.isFMSAttached())
        )

        self.drivetrain.register_telemetry(
            lambda state: self.robot_state.log_swerve_state(state)
        )

    def get_autonomous_command(self) -> commands2.Command:
        return self._auto_chooser.getSelected()