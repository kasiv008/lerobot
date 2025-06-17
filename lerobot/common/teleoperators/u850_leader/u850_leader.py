import logging
import time

from lerobot.common.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.common.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.common.motors.ufactory import XarmMotorBus


from ..teleoperator import Teleoperator
from .config_u850 import U850LeaderConfig

logger = logging.getLogger(__name__)


class U850Leader(Teleoperator):
    """
    U850 Leader Arm designed by TheRobotStudio and Hugging Face.
    """

    config_class = U850LeaderConfig
    name = "u850_leader"

    def __init__(self, config: U850LeaderConfig):
        super().__init__(config)
        self.config = config
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        self.bus = XarmMotorBus(
            port=self.config.port,
            motors={
                "joint1": Motor(1, "ufactory-850-1", MotorNormMode.RANGE_M100_100),
                "joint2": Motor(2, "ufactory-850-2", MotorNormMode.RANGE_M100_100),
                "joint3": Motor(3, "ufactory-850-3", MotorNormMode.RANGE_M100_100),
                "joint4": Motor(4, "ufactory-850-4", MotorNormMode.RANGE_M100_100),
                "joint5": Motor(5, "ufactory-850-5", MotorNormMode.RANGE_0_100),
                "joint6": Motor(6, "ufactory-850-6", MotorNormMode.RANGE_0_100),
                "gripper": Motor(7, "robotiq", MotorNormMode.RANGE_0_100),
            },
        )

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    def connect(self) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.bus.connect()
        # if not self.is_calibrated and calibrate:
        #     self.calibrate()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated
    
    def calibrate(self) -> None:
        return NotImplementedError

    # def calibrate(self) -> None:
    #     logger.info(f"\nRunning calibration of {self}")
    #     self.bus.disable_torque()
    #     for motor in self.bus.motors:
    #         self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

    #     input(f"Move {self} to the middle of its range of motion and press ENTER....")
    #     homing_offsets = self.bus.set_half_turn_homings()

    #     print(
    #         "Move all joints sequentially through their entire ranges "
    #         "of motion.\nRecording positions. Press ENTER to stop..."
    #     )
    #     range_mins, range_maxes = self.bus.record_ranges_of_motion()

    #     self.calibration = {}
    #     for motor, m in self.bus.motors.items():
    #         self.calibration[motor] = MotorCalibration(
    #             id=m.id,
    #             drive_mode=0,
    #             homing_offset=homing_offsets[motor],
    #             range_min=range_mins[motor],
    #             range_max=range_maxes[motor],
    #         )

    #     self.bus.write_calibration(self.calibration)
    #     self._save_calibration()
    #     print(f"Calibration saved to {self.calibration_fpath}")

    def configure(self):
        """Configure the robot after connection."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        
        logger.info(f"Configuring {self}...")
        self.bus.enable()

    # def setup_motors(self) -> None:
    #     for motor in reversed(self.bus.motors):
    #         input(f"Connect the controller board to the '{motor}' motor only and press enter.")
    #         self.bus.setup_motor(motor)
    #         print(f"'{motor}' motor id set to {self.bus.motors[motor].id}")

    def get_action(self) -> dict[str, float]:
        start = time.perf_counter()
        action = self.bus.get_position()
        action = {f"{motor}.pos": val for motor, val in action.items()}
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        # TODO(rcadene, aliberts): Implement force feedback
        raise NotImplementedError

    def disconnect(self) -> None:
        if not self.is_connected:
            DeviceNotConnectedError(f"{self} is not connected.")

        self.bus.disconnect()
        logger.info(f"{self} disconnected.")