import sys
import time
from typing import Union
import multiprocessing as mp

import cv2
import ntcore

# from calibration.CalibrationCommandSource import (CalibrationCommandSource,
#                                                   DirectCalibrationCommandSource)
# from calibration.CalibrationSession import CalibrationSession
from config.config import ConfigStore, LocalConfig, RemoteConfig
from config.ConfigSource import ConfigSource, FileConfigSource, DevConfigSource
from output.OutputPublisher import NTOutputPublisher, OutputPublisher
from output.overlay_util import *
from output.StreamServer import MjpegServer
from pipeline.CameraPoseEstimator import MultiTargetCameraPoseEstimator
from pipeline.Capture import GStreamerCapture, DefaultCapture
from pipeline.FiducialDetector import ArucoFiducialDetector
from pipeline.PoseEstimator import SquareTargetPoseEstimator
from pipeline.Camera import Camera

DEMO_ID = 29

def main(cam: Camera):
        # config = ConfigStore(LocalConfig(), RemoteConfig())
    # local_config_source: ConfigSource = FileConfigSource()
    # remote_config_source: ConfigSource = DevConfigSource()
    # calibration_command_source: CalibrationCommandSource = DirectCalibrationCommandSource()

    # capture = GStreamerCapture()
    # capture = DefaultCapture()
    fiducial_detector = ArucoFiducialDetector(cv2.aruco.DICT_APRILTAG_36h11)
    camera_pose_estimator = MultiTargetCameraPoseEstimator()
    tag_pose_estimator = SquareTargetPoseEstimator()
    # output_publisher: OutputPublisher = NTOutputPublisher()
    # stream_server = MjpegServer(port=8000)
    # calibration_session = CalibrationSession()

    # cam1 = Camera(
    #     local_config_file="vision/northstar/localconfig.json",
    #     remote_config_file="vision/northstar/cam1/cam1_remoteconfig.json",
    #     calib_file="vision/northstar/cam1/cam1_calibration.json",
    #     capture_type="default",
    #     streamport=8000
    # )

    # cam2 = Camera(
    #     local_config_file="vision/northstar/localconfig.json",
    #     remote_config_file="vision/northstar/cam2/cam2_remoteconfig.json",
    #     calib_file="vision/northstar/cam2/cam2_calibration.json",
    #     capture_type="default",
    #     streamport=8001
    # )

    # local_config_source.update(config)
    cam.local_config_source.update(cam.config)

    # ntcore.NetworkTableInstance.getDefault().setServer(config.local_config.server_ip)
    # ntcore.NetworkTableInstance.getDefault().startClient4(config.local_config.device_id)
    # undist_stream_server.start(config)
    # stream_server.start(config)
    # cam.stream_server.start(cam.config)


    frame_count = 0
    last_print = 0
    # was_calibrating = False
    while True:
        cam.remote_config_source.update(cam.config)
        timestamp = time.time()
        success, image = cam.capture.get_frame(cam.config)
        if not success:
            time.sleep(0.5)
            continue

        fps = None
        frame_count += 1
        if time.time() - last_print > 1:
            # time.sleep(4)
            last_print = time.time()
            fps = frame_count
            print("Running at", frame_count, "fps")
            frame_count = 0

        # if calibration_command_source.get_calibrating(config):
        #     # Calibration mode
        #     was_calibrating = True
        #     calibration_session.process_frame(image, calibration_command_source.get_capture_flag(config))

        # elif was_calibrating:
        #     # Finish calibration
        #     calibration_session.finish()
        #     sys.exit(0)

        # elif config.local_config.has_calibration:
        if cam.config.local_config.has_calibration:
            # Normal mode
            image_observations = fiducial_detector.detect_fiducials(image, cam.config)
            [overlay_image_observation(image, x) for x in image_observations]
            camera_pose_observation = camera_pose_estimator.solve_camera_pose(
                [x for x in image_observations if x.tag_id != DEMO_ID], cam.config)
            demo_image_observations = [x for x in image_observations if x.tag_id == DEMO_ID]
            demo_pose_observation: Union[FiducialPoseObservation, None] = None
            if len(demo_image_observations) > 0:
                demo_pose_observation = tag_pose_estimator.solve_fiducial_pose(demo_image_observations[0], cam.config)
            # output_publisher.send(config, timestamp, camera_pose_observation, demo_pose_observation, fps)

        else:
            # No calibration
            print("No calibration found")
            time.sleep(0.5)

        # cam.stream_server.set_frame(image)
        # image = cv2.undistort(image, config.local_config.camera_matrix, config.local_config.distortion_coefficients)


if __name__ == "__main__":
    cam1 = Camera(
        local_config_file="vision/northstar/localconfig.json",
        remote_config_file="vision/northstar/cam1/cam1_remoteconfig.json",
        calib_file="vision/northstar/cam1/cam1_calibration.json",
        capture_type="default",
        streamport=8000
    )

    cam2 = Camera(
        local_config_file="vision/northstar/localconfig.json",
        remote_config_file="vision/northstar/cam2/cam2_remoteconfig.json",
        calib_file="vision/northstar/cam2/cam2_calibration.json",
        capture_type="default",
        streamport=8001
    )

    cameras = [cam1]

    with mp.Pool(processes=len(cameras)) as pool:
        pool.map(main, cameras)