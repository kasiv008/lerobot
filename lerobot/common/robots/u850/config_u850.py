# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from dataclasses import dataclass, field

from lerobot.common.cameras import CameraConfig
from lerobot.common.cameras.opencv import OpenCVCameraConfig
from lerobot.common.cameras.realsense import RealSenseCameraConfig

from ..config import RobotConfig


@RobotConfig.register_subclass("u850_follower")
@dataclass
class u850RobotConfig(RobotConfig):
    # `max_relative_target` limits the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all motors, or a list that is the same length as
    # the number of motors in your follower arms.
    max_relative_target: int | None = None

    port: str = "172.16.0.11"
    # cameras
    #cameras: dict[str, CameraConfig] = field(default_factory=dict)
    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "top": OpenCVCameraConfig(
                index_or_path=6,
                fps=30,
                width=640,
                height=480,
            ),
            "wrist": RealSenseCameraConfig(
                serial_number_or_name="218622273032",
                fps=30,
                width=640,
                height=480,
            ),
        }
    )

    mock: bool = False

    robot_type: str = "u850"

@RobotConfig.register_subclass("u850_follower_end_effector")
@dataclass
class U850FollowerEndEffectorConfig(u850RobotConfig):
    """Configuration for the U850FollowerEndEffector robot."""

    # Default bounds for the end-effector position (in meters)
    end_effector_bounds: dict[str, list[float]] = field(
        default_factory=lambda: {
            "min": [0.3778, -0.4254, -0.1031],  # min x, y, z
            "max": [0.8168, 0.3277, 0.7832],  # max x, y, z
        }
    )

    max_gripper_pos: float = 255

    end_effector_step_sizes: dict[str, float] = field(
        default_factory=lambda: {
            "x": 0.02,
            "y": 0.02,
            "z": 0.02,
        }
    )