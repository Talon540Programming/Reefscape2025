from typing import List

import cv2
from config import ConfigStore
from vision_types import FiducialImageObservation


class FiducialDetector:
    def __init__(self) -> None:
        raise NotImplementedError
    
    def detect_fiducials(self, image: cv2.Mat, config_store: ConfigStore) -> List[FiducialImageObservation]:
        raise NotImplementedError
    

class ArucoFiducialDetector(FiducialDetector):
    def __init__(self, dictionary_id) -> None:
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self._aruco_params = cv2.aruco.DetectorParameters()
    
    def detect_fiducials(self, image: cv2.Mat, config_store: ConfigStore):
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self._aruco_params)
        corners, ids, _ = self.aruco_detector.detectMarkers()
        
    