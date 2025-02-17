# Copyright (c) 2025 FRC 6328
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

import json

import cv2
import numpy
from config.config import ConfigStore, RemoteConfig


class ConfigSource:
    def update(self, config_store: ConfigStore) -> None:
        raise NotImplementedError


class FileConfigSource(ConfigSource):
    def __init__(self, config_filename: str, calibration_filename: str) -> None:
        self._config_filename = config_filename
        self._calibration_filename = calibration_filename
        pass

    def update(self, config_store: ConfigStore) -> None:
        # Get config
        with open(self._config_filename, "r") as config_file:
            config_data = json.loads(config_file.read())
            config_store.local_config.device_id = config_data["device_id"]
            config_store.local_config.server_ip = config_data["server_ip"]
            config_store.local_config.apriltags_stream_port = config_data["apriltags_stream_port"]
            config_store.local_config.objdetect_stream_port = config_data["objdetect_stream_port"]
            config_store.local_config.capture_impl = config_data["capture_impl"]
            config_store.local_config.obj_detect_model = config_data["obj_detect_model"]
            config_store.local_config.obj_detect_max_fps = config_data["obj_detect_max_fps"]
            config_store.local_config.apriltags_enable = config_data["apriltags_enable"]
            config_store.local_config.objdetect_enable = config_data["objdetect_enable"]
            config_store.local_config.video_folder = config_data["video_folder"]

        # Get calibration
        calibration_store = cv2.FileStorage(self._calibration_filename, cv2.FILE_STORAGE_READ)
        camera_matrix = calibration_store.getNode("camera_matrix").mat()
        distortion_coefficients = calibration_store.getNode("distortion_coefficients").mat()
        calibration_store.release()
        if type(camera_matrix) == numpy.ndarray and type(distortion_coefficients) == numpy.ndarray:
            config_store.local_config.camera_matrix = camera_matrix
            config_store.local_config.distortion_coefficients = distortion_coefficients
            config_store.local_config.has_calibration = True
