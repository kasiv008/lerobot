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


@RobotConfig.register_subclass("u850")
@dataclass
class u850RobotConfig(RobotConfig):
    # `max_relative_target` limits the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all motors, or a list that is the same length as
    # the number of motors in your follower arms.
    max_relative_target: int | None = None

    port: str = "172.16.0.11"
    # cameras
    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "top": OpenCVCameraConfig(
                index_or_path=6,
                fps=30,
                width=640,
                height=480,
            ),
            "wrist": RealSenseCameraConfig(
                name="Intel RealSense D405",
                fps=30,
                width=640,
                height=480,
            ),
        }
    )

    mock: bool = False
