package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.LinkedList;
import java.util.List;
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

    // Read new camera observations
    List<PoseObservation> poseObservations = new LinkedList<>();

    // Process camera results
    for (var result : camera.getAllUnreadResults()) {
      var observationBuilder = PoseObservation.builder().timestamp(result.getTimestampSeconds());

      var multitagResOpt = result.getMultiTagResult();
      if (multitagResOpt.isPresent()) {
        var multitagRes = multitagResOpt.get();

        observationBuilder
            .multitagTagToCamera(multitagRes.estimatedPose.best)
            .ambiguity(multitagRes.estimatedPose.ambiguity)
            .detectedTagsIds(
                multitagRes.fiducialIDsUsed.stream().mapToInt(Short::shortValue).toArray());
      } else if (result.hasTargets()) {
        var singletagDetection = result.getBestTarget();

        observationBuilder
            .bestTagToCamera(singletagDetection.bestCameraToTarget)
            .altTagToCamera(singletagDetection.altCameraToTarget)
            .ambiguity(singletagDetection.poseAmbiguity)
            .detectedTagsIds(
                result.getTargets().stream()
                    .mapToInt(PhotonTrackedTarget::getFiducialId)
                    .toArray());
      }

      poseObservations.add(observationBuilder.build());
    }

    inputs.observations = poseObservations.toArray(new PoseObservation[0]);
  }
}
