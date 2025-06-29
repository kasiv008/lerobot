import logging
import time
from functools import cached_property
from typing import Any

from lerobot.common.cameras.utils import make_cameras_from_configs
from lerobot.common.constants import OBS_STATE
from lerobot.common.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.common.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.common.motors.ufactory import XarmMotorBus
from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_u850 import u850RobotConfig

logger = logging.getLogger(__name__)

class U850Robot(Robot):
    """
    [U850](https://www.ur3d.com/u850) developed by UFactory
    """

    config_class = u850RobotConfig
    name = "u850"

    def __init__(self, config: u850RobotConfig):
        super().__init__(config)
        self.config = config
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
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _state_ft(self) -> dict[str, type]:
        return dict.fromkeys(
            (
                "joint1.pos",
                "joint2.pos",
                "joint3.pos",
                "joint4.pos",
                "joint5.pos",
                "joint6.pos",
                "gripper.pos",
            ),
            float,
        )
    
    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())
    
    def connect(self, calibrate: bool = True) -> None:
        """
        We assume that at connection time, arm is in a rest position,
        and torque can be safely disabled to run calibration.
        """
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.bus.connect()
        # if not self.is_calibrated and calibrate:
        #     self.calibrate()

        for cam in self.cameras.values():
            cam.connect()

        self.configure()
        logger.info(f"{self} connected.")
    
    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated
    
    def calibrate(self) -> None:
        return NotImplementedError

    def configure(self):
        """Configure the robot after connection."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        
        logger.info(f"Configuring {self}...")
        _ = self.bus.enable(follower=True, return_init_pos=True)

    def get_observation(self) -> dict[str, Any]:
        """The returned observations do not have a batch dimension."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        obs_dict = {}

        # Read arm position
        start = time.perf_counter()
        obs_dict = self.bus.get_position()
        obs_dict = {f"{motor}.pos": val for motor, val in obs_dict.items()}
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict
    
    def send_action(self, action: dict[str, float]) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(
                "ManipulatorRobot is not connected. You need to run `robot.connect()`."
            )
        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}

        # Cap goal position when too far away from present position.
        # /!\ Slower fps expected due to reading from the follower.
        if self.config.max_relative_target is not None:
            present_pos = self.bus.get_position()
            goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in goal_pos.items()}
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        # Send goal position to the arm
        self.bus.set_position(goal_pos)
        return {f"{motor}.pos": val for motor, val in goal_pos.items()}
    
    def get_ee_coordinates(self) -> dict[str, float]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        # Read arm position
        start = time.perf_counter()
        ee_pos = self.bus.get_cartesian_position()
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read ee coordinates: {dt_ms:.1f}ms")
        return ee_pos

    
    def disconnect(self):
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.bus.disconnect()
        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")