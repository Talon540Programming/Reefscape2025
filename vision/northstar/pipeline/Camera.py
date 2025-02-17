from pipeline.Capture import GStreamerCapture, DefaultCapture
from output.StreamServer import MjpegServer
from config.ConfigSource import ConfigStore, DevConfigSource, FileConfigSource, ConfigSource
from config.config import LocalConfig, RemoteConfig

class Camera:
    def __init__(self, local_config_file, remote_config_file, calib_file, capture_type="gstreamer", streamport=8000):
        self.capture = GStreamerCapture() if capture_type == "gstreamer" else DefaultCapture()

        self.config = ConfigStore(LocalConfig(), RemoteConfig())
        self.local_config_source: ConfigSource = FileConfigSource(local_config_file, calib_file)
        self.remote_config_source: ConfigSource = DevConfigSource(remote_config_file)

        self.stream_server = MjpegServer(port=streamport)