import math
from enum import Enum, auto

from commands2 import Command
from commands2.sysid import SysIdRoutine
from phoenix6 import SignalLogger, BaseStatusSignal
from phoenix6.configs import TalonFXConfiguration, MotorOutputConfigs, FeedbackConfigs, CANdiConfiguration, HardwareLimitSwitchConfigs
from phoenix6.configs.config_groups import NeutralModeValue, MotionMagicConfigs
from phoenix6.controls import Follower, VoltageOut, DutyCycleOut, MotionMagicDutyCycle
from phoenix6.hardware import CANdi, TalonFX
from phoenix6.signals import ForwardLimitSourceValue
from wpilib import DriverStation
from wpilib.sysid import SysIdRoutineLog
from wpimath.filter import Debouncer
from wpimath.system.plant import DCMotor

from constants import Constants
from subsystems import StateSubsystem


class ElevatorSubsystem(StateSubsystem):
    """
    The ElevatorSubsystem is responsible for controlling the elevator mechanism.
    It manages motor positions and uses Motion Magic to smoothly transition between set states.

    It also reads values passed from the CANdi S1 and S2 inputs to ensure the elevator stays within safe bounds.
    """

    class SubsystemState(Enum):
        IDLE = auto()
        DEFAULT = auto()
        L1 = auto()
        L2 = auto()
        L3 = auto()
        L4 = auto()
        L2_ALGAE = auto()
        L3_ALGAE = auto()
        NET = auto()

    _candi_config = CANdiConfiguration()

    _motor_config = (TalonFXConfiguration()
                     .with_slot0(Constants.ElevatorConstants.GAINS)
                     .with_motor_output(MotorOutputConfigs().with_neutral_mode(NeutralModeValue.BRAKE))
                     .with_feedback(FeedbackConfigs().with_sensor_to_mechanism_ratio(Constants.ElevatorConstants.GEAR_RATIO))
                     .with_motion_magic(MotionMagicConfigs().with_motion_magic_acceleration(6).with_motion_magic_cruise_velocity(6))
                     )

    # Limit switch config (separate since it's only applied to the master motor)
    ### NOTE: Flip positions when inverting motor output
    _limit_switch_config = HardwareLimitSwitchConfigs()
    _limit_switch_config.forward_limit_remote_sensor_id = Constants.CanIDs.ELEVATOR_CANDI
    _limit_switch_config.forward_limit_source = ForwardLimitSourceValue.REMOTE_CANDIS2 # Bottom Limit Switch
    _limit_switch_config.forward_limit_autoset_position_value = Constants.ElevatorConstants.DEFAULT_POSITION
    _limit_switch_config.forward_limit_autoset_position_enable = True

    _limit_switch_config.reverse_limit_remote_sensor_id = Constants.CanIDs.ELEVATOR_CANDI
    _limit_switch_config.reverse_limit_source = ForwardLimitSourceValue.REMOTE_CANDIS1 # Top Limit Switch
    _limit_switch_config.reverse_limit_autoset_position_value = Constants.ElevatorConstants.ELEVATOR_MAX
    _limit_switch_config.reverse_limit_autoset_position_enable = True

    def __init__(self) -> None:
        super().__init__("Elevator")

        self._master_motor = TalonFX(Constants.CanIDs.LEFT_ELEVATOR_TALON)
        _master_config = self._motor_config
        _master_config.hardware_limit_switch = self._limit_switch_config
        self._master_motor.configurator.apply(self._motor_config)

        self._follower_motor = TalonFX(Constants.CanIDs.RIGHT_ELEVATOR_TALON)
        self._follower_motor.configurator.apply(self._motor_config)

        self._candi = CANdi(Constants.CanIDs.ELEVATOR_CANDI)
        self._candi.configurator.apply(self._candi_config)

        self._position_request = MotionMagicDutyCycle(0)
        self._brake_request = DutyCycleOut(0)
        self._sys_id_request = VoltageOut(0)

        self._master_motor.set_control(self._brake_request)
        self._follower_motor.set_control(Follower(self._master_motor.device_id, False))

        self._add_talon_sim_model(self._master_motor, DCMotor.krakenX60FOC(2), Constants.ElevatorConstants.GEAR_RATIO)

        self._at_setpoint_debounce = Debouncer(0.1, Debouncer.DebounceType.kRising)
        self._at_setpoint = True

        self._master_motor.set_position(Constants.ElevatorConstants.DEFAULT_POSITION)

        self._sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdElevator_State", SysIdRoutineLog.stateEnumToString(state)
                )  # Log to .hoot for ease of access
            ),
            SysIdRoutine.Mechanism(
                lambda output: self._master_motor.set_control(self._sys_id_request.with_output(output)),
                lambda log: None,
                self,
            )
        )

    def periodic(self) -> None:
        super().periodic()

        latency_compensated_position = BaseStatusSignal.get_latency_compensated_value(
            self._master_motor.get_position(), self._master_motor.get_velocity()
        )
        self._at_setpoint = self._at_setpoint_debounce.calculate(abs(latency_compensated_position - self._position_request.position) <= Constants.ElevatorConstants.SETPOINT_TOLERANCE)
        self.get_network_table().getEntry("At Setpoint").setBoolean(self._at_setpoint)

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        if DriverStation.isTest() or self.is_frozen():
            return

        match self._subsystem_state:
            case self.SubsystemState.IDLE:
                pass
            case self.SubsystemState.DEFAULT:
                self._position_request.position = Constants.ElevatorConstants.DEFAULT_POSITION
            case self.SubsystemState.L1:
                self._position_request.position = Constants.ElevatorConstants.L1_SCORE_POSITION
            case self.SubsystemState.L2:
                self._position_request.position = Constants.ElevatorConstants.L2_SCORE_POSITION
            case self.SubsystemState.L3:
                self._position_request.position = Constants.ElevatorConstants.L3_SCORE_POSITION
            case self.SubsystemState.L4:
                self._position_request.position = Constants.ElevatorConstants.L4_SCORE_POSITION
            case self.SubsystemState.L2_ALGAE:
                self._position_request.position = Constants.ElevatorConstants.L2_ALGAE_POSITION
            case self.SubsystemState.L3_ALGAE:
                self._position_request.position = Constants.ElevatorConstants.L3_ALGAE_POSITION
            case self.SubsystemState.NET:
                self._position_request.position = Constants.ElevatorConstants.NET_SCORE_POSITION

        self._subsystem_state = desired_state

        if desired_state is not self.SubsystemState.IDLE:
            self._master_motor.set_control(self._position_request)
        else:
            self._master_motor.set_control(self._brake_request)

    def is_at_setpoint(self) -> bool:
        return self._at_setpoint

    def stop(self) -> Command:
        return self.runOnce(lambda: self._master_motor.set_control(self._brake_request))

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self._sys_id_routine.quasistatic(direction).andThen(self.stop())

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self._sys_id_routine.dynamic(direction).andThen(self.stop())

    def get_height(self) -> float:
        """Returns the height of the elevator, in meters."""
        return (self._master_motor.get_position().value / Constants.ElevatorConstants.GEAR_RATIO) * (2 * math.pi * 0.508)