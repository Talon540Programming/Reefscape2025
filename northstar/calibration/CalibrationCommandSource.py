# Copyright (c) 2025 FRC 6328
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

from config.config import ConfigStore


class CalibrationCommandSource:
    def get_calibrating(self) -> bool:
        return False

    def get_capture_flag(self) -> bool:
        return False


class NTCalibrationCommandSource(CalibrationCommandSource):
    _init_complete: bool = False
    _active_entry: bool = False
    _capture_flag_entry: bool = False

    def _init(self, config_store: ConfigStore):
        if not self._init_complete:

            self._active_entry = False
            self._capture_flag_entry = False
            self._init_complete = True

    def get_calibrating(self, config_store: ConfigStore) -> bool:
        self._init(config_store)
        calibrating = self._active_entry
        if not calibrating:
            self._capture_flag_entry = False
        return calibrating

    def get_capture_flag(self, config_store: ConfigStore) -> bool:
        self._init(config_store)
        if self._capture_flag_entry:
            self._capture_flag_entry = False
            return True
        return False
