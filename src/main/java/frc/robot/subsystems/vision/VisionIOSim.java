package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import java.io.IOException;
import java.nio.file.Path;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOSim extends VisionIOPhotonCamera {
  private static VisionSystemSim m_visionSystemSim;

  static {
    if (Constants.getRobot() == Constants.RobotType.SIMBOT) {
      m_visionSystemSim = new VisionSystemSim("main");
      m_visionSystemSim.addAprilTags(FieldConstants.defaultAprilTagType.getLayout());
    }
  }

  public VisionIOSim(int index) {
    super(index);
    var cameraProps = new SimCameraProperties();
    // Photon camera sim is currently out of date with the calibration export model, so we can only
    // pull the camera intrinsics and distortion from it
    try {
      setCalibrationFromConfig(cameras[index].calibrationPath(), 1280, 800, cameraProps);
    } catch (IOException e) {
      throw new RuntimeException("Unable to parse calibration data from file");
    }

    // These are per camera but this average is good enough
    cameraProps.setCalibError(.30, 0.05);
    cameraProps.setFPS(50);
    cameraProps.setAvgLatencyMs(25);
    cameraProps.setLatencyStdDevMs(10);

    var camSim = new PhotonCameraSim(this.camera, cameraProps);
    camSim.enableRawStream(false);
    camSim.enableProcessedStream(false);
    camSim.enableDrawWireframe(false);

    m_visionSystemSim.addCamera(camSim, cameras[index].robotToCamera());
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    m_visionSystemSim.update(RobotState.getInstance().getEstimatedPose());
    super.updateInputs(inputs);
  }

  private void setCalibrationFromConfig(
      Path path, int resWidth, int resHeight, SimCameraProperties properties)
      throws IOException, IllegalStateException {
    var mapper = new ObjectMapper();
    var calibData = mapper.readTree(path.toFile());
    int jsonWidth = calibData.get("resolution").get("width").asInt();
    int jsonHeight = calibData.get("resolution").get("height").asInt();
    if (jsonWidth != resWidth || jsonHeight != resHeight) {
      throw new IllegalStateException(
          "The provided calibration file doesn't match the requested resolution");
    }

    var jsonIntrinsicsNode = calibData.get("cameraIntrinsics").get("data");
    double[] jsonIntrinsics = new double[jsonIntrinsicsNode.size()];
    for (int j = 0; j < jsonIntrinsicsNode.size(); j++) {
      jsonIntrinsics[j] = jsonIntrinsicsNode.get(j).asDouble();
    }
    var jsonDistortNode = calibData.get("distCoeffs").get("data");
    // Calibration model only needs to include first five elements of the distortion vector.
    // https://discord.com/channels/725836368059826228/725848198794706994/1210448658936365076
    int numCol = Math.min(8, jsonDistortNode.size());
    double[] jsonDistortion = new double[numCol];
    for (int j = 0; j < numCol; j++) {
      jsonDistortion[j] = jsonDistortNode.get(j).asDouble();
    }

    properties.setCalibration(
        jsonWidth,
        jsonHeight,
        MatBuilder.fill(Nat.N3(), Nat.N3(), jsonIntrinsics),
        MatBuilder.fill(Nat.N8(), Nat.N1(), jsonDistortion));
  }
}
