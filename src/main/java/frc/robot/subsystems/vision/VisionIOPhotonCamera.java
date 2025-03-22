package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhotonCamera implements VisionIO {
  protected final PhotonCamera camera;

  public VisionIOPhotonCamera(int index) {
    camera = new PhotonCamera(cameras[index].cameraName());
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.ntConnected = camera.isConnected();

    // Process camera results
    var resultQueue = camera.getAllUnreadResults();
    // Read new camera observations
    inputs.observations = new PoseObservation[resultQueue.size()];
    inputs.detectedTagIds = new int[resultQueue.size()][];
    for (int i = 0; i < resultQueue.size(); i++) {
      var result = resultQueue.get(i);

      var observationBuilder =
          PoseObservation.builder()
              .timestamp(result.getTimestampSeconds())
              .multitagTagToCamera(new Transform3d())
              .bestTagToCamera(new Transform3d())
              .altTagToCamera(new Transform3d());

      var multitagResOpt = result.getMultiTagResult();
      if (multitagResOpt.isPresent()) {
        var multitagRes = multitagResOpt.get();

        observationBuilder
            .hasResult(true)
            .isMultitag(true)
            .multitagTagToCamera(multitagRes.estimatedPose.best)
            .ambiguity(multitagRes.estimatedPose.ambiguity);

        inputs.detectedTagIds[i] =
            multitagRes.fiducialIDsUsed.stream().mapToInt(Short::shortValue).toArray();
      } else if (result.hasTargets()) {
        var singletagDetection = result.getBestTarget();

        observationBuilder
            .hasResult(true)
            .bestTagToCamera(singletagDetection.bestCameraToTarget)
            .altTagToCamera(singletagDetection.altCameraToTarget)
            .ambiguity(singletagDetection.poseAmbiguity);
        inputs.detectedTagIds[i] =
            result.getTargets().stream().mapToInt(PhotonTrackedTarget::getFiducialId).toArray();
      }

      inputs.observations[i] = observationBuilder.build();
    }
  }
}
