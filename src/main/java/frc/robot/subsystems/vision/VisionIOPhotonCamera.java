package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.FieldConstants;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.PhotonCamera;

public class VisionIOPhotonCamera implements VisionIO {
  protected final PhotonCamera camera;
  private final int camIndex;
  private final StringPublisher atflPublisher;

  public VisionIOPhotonCamera(int index) {
    camera = new PhotonCamera(cameras[index].cameraName());
    camIndex = index;
    atflPublisher =
        NetworkTableInstance.getDefault()
            .getTable(PhotonCamera.kTableName)
            .getStringTopic("apriltag_field_layout")
            .publish();
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.ntConnected = camera.isConnected();

    // Process camera results
    var resultQueue = camera.getAllUnreadResults();
    // Read new camera observations
    inputs.observations = new PoseObservation[resultQueue.size()];
    for (int i = 0; i < resultQueue.size(); i++) {
      var result = resultQueue.get(i);

      var observationBuilder =
          PoseObservation.builder().timestampSeconds(result.getTimestampSeconds());

      var multitagResOpt = result.getMultiTagResult();
      if (multitagResOpt.isPresent()) {
        var multitagRes = multitagResOpt.get();
        observationBuilder
            .hasResult(true)
            .multitagResult(
                Optional.of(new MultitagPoseObservation(multitagRes.estimatedPose.best)))
            .detectedTagIds(multitagRes.fiducialIDsUsed);
      } else if (result.hasTargets()) {
        var singleTagDetection = result.getBestTarget();

        observationBuilder
            .hasResult(true)
            .singleTagResult(
                Optional.of(
                    new SingleTagPoseObservation(
                        singleTagDetection.fiducialId,
                        singleTagDetection.bestCameraToTarget,
                        singleTagDetection.altCameraToTarget,
                        singleTagDetection.poseAmbiguity,
                        Rotation2d.fromDegrees(-singleTagDetection.pitch),
                        Rotation2d.fromDegrees(-singleTagDetection.yaw))));

        var tagList = new ArrayList<Short>();
        for (var res : result.getTargets()) {
          tagList.add((short) res.getFiducialId());
        }
        observationBuilder.detectedTagIds(tagList);
      }

      inputs.observations[i] = observationBuilder.build();
    }
  }

  @Override
  public void setAprilTagFieldLayout(FieldConstants.AprilTagLayoutType layoutType) {
    atflPublisher.set(layoutType.getLayoutString());
  }

  @Override
  public int getCamIndex() {
    return camIndex;
  }
}
