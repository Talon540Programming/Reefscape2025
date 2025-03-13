package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.FieldConstants;
import java.util.ArrayList;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhotonCamera implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  private final Matrix<N3, N1> cameraBias;

  private static final Matrix<N3, N1> INVALID_STDDEVS = VecBuilder.fill(-1.0, -1.0, -1.0);
  private static final Matrix<N3, N1> SINGLE_TARGET_STDDEVS =
      VecBuilder.fill(0.1, 0.1, Math.toRadians(2.5));
  private static final Matrix<N3, N1> MULTI_TARGET_STDDEVS =
      VecBuilder.fill(0.01, 0.01, Math.toRadians(0.5));

  // Single tag the lowest ambiguity tags must be below the threshold
  // Robot pose estimates on the side of the field may be slightly off, this accounts for that
  // AprilTag estimation may result in 3d estimations with poor z estimates, reject estimates with
  // too crazy z estimates

  public VisionIOPhotonCamera(
      String cameraName, Transform3d robotToCamera, Matrix<N3, N1> cameraBias) {
    camera = new PhotonCamera(cameraName);
    this.robotToCamera = robotToCamera;
    this.cameraBias = cameraBias;
  }

  public Transform3d getRobotToCamera() {
    return robotToCamera;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.isConnected = camera.isConnected();

    var resList = camera.getAllUnreadResults();
    if (!resList.isEmpty()) {
      PhotonPipelineResult res = resList.get(resList.size() - 1);
      // var res = camera.getLatestResult();

      // for (PhotonPipelineResult res : resList) {
      inputs.timestampSeconds = res.getTimestampSeconds();
      inputs.hasResult = false;

      if (res.getMultiTagResult().isPresent()) {

        Transform3d fieldToCamera = res.getMultiTagResult().get().estimatedPose.best;
        inputs.hasResult = true;
        inputs.estimatedRobotPose =
            new Pose3d()
                .plus(fieldToCamera)
                .relativeTo(FieldConstants.fieldLayout.getOrigin())
                .plus(robotToCamera.inverse());
        inputs.detectedTagsIds =
            res.getMultiTagResult().get().fiducialIDsUsed.stream()
                .mapToInt(Short::intValue)
                .toArray();

        inputs.visionMeasurementStdDevs = MULTI_TARGET_STDDEVS;
      } else if (!res.getTargets().isEmpty()) {

        // Find the detected target with the lowest pose ambiguity that is within the field
        double lowestAmbiguityScore = Double.POSITIVE_INFINITY;
        PhotonTrackedTarget lowestAmbiguityTarget = null;
        var tagsUsed = new ArrayList<Integer>();
        for (var target : res.getTargets()) {
          double targetPoseAmbiguity = target.getPoseAmbiguity();

          //   Filter out tags that aren't fiducial targets (IDK how) or in the ATFL
          if (targetPoseAmbiguity == -1
              || FieldConstants.fieldLayout.getTagPose(target.getFiducialId()).isEmpty()) continue;

          tagsUsed.add(target.getFiducialId());

          if (targetPoseAmbiguity < lowestAmbiguityScore
              && targetPoseAmbiguity <= ambiguityThreshold) {
            lowestAmbiguityScore = targetPoseAmbiguity;
            lowestAmbiguityTarget = target;
          }
        }

        if (lowestAmbiguityTarget != null) {
          var tagPoseOpt =
              FieldConstants.fieldLayout.getTagPose(lowestAmbiguityTarget.getFiducialId());
          if (tagPoseOpt.isPresent()) {
            inputs.hasResult = true;
            inputs.estimatedRobotPose =
                tagPoseOpt
                    .get()
                    .transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse())
                    .transformBy(robotToCamera.inverse());
            inputs.detectedTagsIds = tagsUsed.stream().mapToInt(Integer::intValue).toArray();
            inputs.visionMeasurementStdDevs = SINGLE_TARGET_STDDEVS;
          }
        }
      }

      // Reject pose estimates outside reasonable bounds
      if (!inputs.hasResult
          || inputs.estimatedRobotPose.getX() < -fieldBorderMargin
          || inputs.estimatedRobotPose.getX()
              > FieldConstants.fieldLayout.getFieldLength() + fieldBorderMargin
          || inputs.estimatedRobotPose.getY() < -fieldBorderMargin
          || inputs.estimatedRobotPose.getY()
              > FieldConstants.fieldLayout.getFieldWidth() + fieldBorderMargin
          || inputs.estimatedRobotPose.getZ() < -zMargin
          || inputs.estimatedRobotPose.getZ() > zMargin) {
        // Record default values if no results or invalid were found
        inputs.hasResult = false;
        inputs.estimatedRobotPose = new Pose3d();
        inputs.detectedTagsIds = new int[] {};
        inputs.detectedTagPoses = new Pose3d[] {};
        inputs.visionMeasurementStdDevs = INVALID_STDDEVS;
        inputs.tagDistances = new double[] {};
      } else {
        // Calculate the average distance between the estimated pose and the pose of all detected
        // fiducial targets in three dimensions
        double totalDistance = 0.0;
        // Guaranteed to have at least one target
        int numTags = inputs.detectedTagsIds.length;
        inputs.detectedTagPoses = new Pose3d[numTags];
        inputs.tagDistances = new double[numTags];
        for (int i = 0; i < numTags; i++) {
          int tagId = inputs.detectedTagsIds[i];
          // All values are guaranteed to exist within the ATFL
          var tagPose = FieldConstants.fieldLayout.getTagPose(tagId).orElseThrow();
          inputs.detectedTagPoses[i] = tagPose;
          inputs.tagDistances[i] =
              tagPose.getTranslation().getDistance(inputs.estimatedRobotPose.getTranslation());
          totalDistance += inputs.tagDistances[i];
        }
        double averageDistance = totalDistance / numTags;
        // Increase std devs based on the average distance to the target and per camera bias
        inputs.visionMeasurementStdDevs =
            inputs
                .visionMeasurementStdDevs
                .times(1.0 + ((Math.pow(averageDistance, 2.0) / numTags)))
                .elementTimes(cameraBias);
      }
    }
  }
}
