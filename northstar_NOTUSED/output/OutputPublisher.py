# Copyright (c) 2025 FRC 6328
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

import math
from typing import List, Union

from config.config import ConfigStore
from vision_types import CameraPoseObservation, FiducialPoseObservation, ObjDetectObservation, TagAngleObservation


class OutputPublisher:
    def send_apriltag_fps(self, config_store: ConfigStore, timestamp: float, fps: int) -> None:
        raise NotImplementedError

    def send_apriltag_observation(
        self,
        config_store: ConfigStore,
        timestamp: float,
        observation: Union[CameraPoseObservation, None],
        tag_angles: List[TagAngleObservation],
        demo_observation: Union[FiducialPoseObservation, None],
    ) -> None:
        raise NotImplementedError

