import logging
import time
from functools import cached_property
from typing import Any

from lerobot.common.cameras.utils import make_cameras_from_configs
from lerobot.common.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.common.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.common.motors.ufactory import XarmMotorBus
from .config_u850 import U850FollowerEndEffectorConfig
from . import U850Robot
import numpy as np

logger = logging.getLogger(__name__)

class U850FollowerEndEffector(U850Robot):
    """
    [U850](https://www.ur3d.com/u850) developed by UFactory
    """

    config_class = U850FollowerEndEffectorConfig
    name = "u850_follower_end_effector"

    def __init__(self, config: U850FollowerEndEffectorConfig):
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

        # Store the bounds for end-effector position
        self.end_effector_bounds = self.config.end_effector_bounds

        self.current_ee_pos = None
        self.current_joint_pos = None

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
    
    @cached_property
    def action_features(self) -> dict[str, type]:
        return {
            "dtype": "float32",
            "shape": (4,),
            "names": {"delta_x": 0, "delta_y": 1, "delta_z": 2, "gripper": 3},
        }



    def get_observation(self) -> dict[str, Any]:
        """The returned observations do not have a batch dimension."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

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
        if isinstance(action, dict):
            if all(k in action for k in ["delta_x", "delta_y", "delta_z"]):
                delta_ee = np.array(
                    [
                        action["delta_x"] * self.config.end_effector_step_sizes["x"],
                        action["delta_y"] * self.config.end_effector_step_sizes["y"],
                        action["delta_z"] * self.config.end_effector_step_sizes["z"],
                    ],
                    dtype=np.float32,
                )
                if "gripper" not in action:
                    action["gripper"] = [1.0]
                action = np.append(delta_ee, action["gripper"])
            else:
                logger.warning(
                    f"Expected action keys 'delta_x', 'delta_y', 'delta_z', got {list(action.keys())}"
                )
                action = np.zeros(4, dtype=np.float32)
        
        if self.current_joint_pos is None:
            # Read current joint positions
            current_joint_pos = self.bus.get_position()
            self.current_joint_pos = np.array([current_joint_pos[name] for name in self.bus.motor_names])

        if self.current_ee_pos is None:
            # Read current end-effector position
            self.current_ee_pos = self.bus.get_forward_kinematics()

        # Set desired end-effector position by adding delta
        desired_ee_pos = np.eye(4)
        desired_ee_pos[:3, :3] = self.current_ee_pos[:3, :3]  # Keep orientation

        # Add delta to position and clip to bounds
        desired_ee_pos[:3, 3] = self.current_ee_pos[:3, 3] + action[:3]
        if self.end_effector_bounds is not None:
            desired_ee_pos[:3, 3] = np.clip(
                desired_ee_pos[:3, 3],
                self.end_effector_bounds["min"],
                self.end_effector_bounds["max"],
            )

        gripper_pos = int(np.clip(
            self.current_joint_pos[-1] + (action[-1] - 1) * self.config.max_gripper_pos,
            0,
            self.config.max_gripper_pos,
        ))

        x, y, z = desired_ee_pos[:3, 3]*1000  # Convert to mm
        pitch = np.arcsin(-desired_ee_pos[2, 0])
        if np.cos(pitch) > 1e-10:
            roll = np.arctan2(desired_ee_pos[2, 1], desired_ee_pos[2, 2])
            yaw = np.arctan2(desired_ee_pos[1, 0], desired_ee_pos[0, 0])
        else:
            roll = np.arctan2(-desired_ee_pos[1, 2], desired_ee_pos[1, 1])
            yaw = 0
        desired_cart_pos = np.array([x, y, z, np.degrees(roll), np.degrees(pitch), np.degrees(yaw)])

        self.bus.set_cartesian_position(desired_cart_pos)
        self.bus.set_gripper_pos(gripper_pos)
        
        self.current_ee_pos = desired_ee_pos
        self.current_joint_pos = np.append(self.current_joint_pos[:-1], gripper_pos)    


    def get_ee_coordinates(self) -> dict[str, float]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        # Read arm position
        start = time.perf_counter()
        ee_pos = self.bus.get_cartesian_position()
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read ee coordinates: {dt_ms:.1f}ms")
        return ee_pos
    
    def reset(self):
        self.current_ee_pos = None
        self.current_joint_pos = None 