import argparse
import queue
import sys
import threading
import time
from typing import List, Tuple, Union
import cv2

from networktables import NetworkTables

if __name__ == "__main__":
    apriltags_frame_count = 0
    apriltags_last_print = 0
    was_calibrating = False
    was_recording = False

    last_image_observations: List = [] #TODO
    video_frame_cache: List[cv2.Mat] = []

    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()

        if not ret:
            print("End of video stream")
            break

        cv2.imshow('Video', frame)

        if cv2.waitKey(25) & 0xFF == ord('q'): # Exit on pressing 'q'
            break