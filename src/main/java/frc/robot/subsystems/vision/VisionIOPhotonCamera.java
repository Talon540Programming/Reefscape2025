package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.PhotonCamera;

public class VisionIOPhotonCamera implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  private final Matrix<N3, N1> cameraBias;

  private int pipeline = 0;

  private static final Matrix<N3, N1> INVALID_STDDEVS = VecBuilder.fill(-1.0, -1.0, -1.0);
  private static final Matrix<N3, N1> SINGLE_TARGET_STDDEVS =
      VecBuilder.fill(0.1, 0.1, Math.toRadians(2.5));
  private static final Matrix<N3, N1> MULTI_TARGET_STDDEVS =
      VecBuilder.fill(0.01, 0.01, Math.toRadians(0.5));

  // Single tag the lowest ambiguity tags must be below the threshold
  private static final double AMBIGUITY_THRESHOLD = 0.4;
  // Robot pose estimates on the side of the field may be slightly off, this accounts for that
  private static final double FIELD_BORDER_MARGIN = 0.5;
  // AprilTag estimation may result in 3d estimations with poor z estimates, reject estimates with
  // too crazy z estimates
  private static final double Z_MARGIN = 0.75;

  public VisionIOPhotonCamera(
      String cameraName, Transform3d robotToCamera, Matrix<N3, N1> cameraBias) {
    camera = new PhotonCamera(cameraName);
    this.robotToCamera = robotToCamera;
    this.cameraBias = cameraBias;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.isConnected = camera.isConnected();
    // inputs.pipelineIndex = pipeline;
    // System.out.println(inputs.pipelineIndex);

    // if (inputs.pipelineIndex == 0) {
    //   updateAprilTag(inputs);
    // } else if (inputs.pipelineIndex == 1) {
    updateCorners(inputs);
    // } else {
    //   System.out.println("Pipeline index not found");
    // }
  }

  @Override
  public void updateCorners(VisionIOInputs inputs) {
    var resList = camera.getAllUnreadResults();

    if (!resList.isEmpty()) {
      var res = resList.get(resList.size() - 1);

      inputs.timestampSeconds = res.getTimestampSeconds();

      inputs.hasResult = false;

      if (!res.getTargets().isEmpty()) {

        // inputs.detectedCorners = res.getTargets().stream().map(
        //   t -> t.getDetectedCorners().stream().map(
        //     tc -> new Translation2d(tc.x, tc.y)
        //   ).toArray(Translation2d[]::new)
        // ).toArray(Translation2d[][]::new);

        // inputs.detectedCorners = res.getTargets().stream().map(
        //   t -> t.getDetectedCorners().
        // )

        inputs.hasResult = true;
        var bestTarget = res.getBestTarget();

        inputs.detectedTagsIds = new int[] {res.getBestTarget().fiducialId};
        inputs.detectedCorners =
            bestTarget.getDetectedCorners().stream()
                .map(tc -> new Translation2d(tc.x, tc.y))
                .toArray(Translation2d[]::new);

        double centerx = 0.0;

        for (int i = 0; i < 4; i++) {
          centerx += inputs.detectedCorners[i].getX();
        }

        centerx /= 4;

        inputs.tagCenterX = centerx;

        double hypotDistance = bestTarget.getBestCameraToTarget().getX();
        inputs.tagHorizontalDistance = hypotDistance;

        inputs.tagHorizontalDistance =
            hypotDistance * Math.cos(-robotToCamera.getRotation().getY());

        // for (PhotonTrackedTarget target : res.getTargets()) {
        //   // var detectedCorners = target.getDetectedCorners();
        //   inputs.detectedCorners = target.getDetectedCorners().stream().map(
        //     tc -> new Translation2d(tc.x, tc.y)
        //   ).toArray(Translation2d[]::new);
        // }
      }

      if (!inputs.hasResult) {
        inputs.detectedCorners = new Translation2d[] {};
        // inputs.detectedTagPoses = new Pose3d[] {};
        inputs.tagHorizontalDistance = 0;
        inputs.tagCenterX = Integer.MIN_VALUE;
      }
    }
  }

  @Override
  public Transform3d getRobotToCamera() {
    return robotToCamera;
  }

  // @Override
  // public void setPipelineIndex(int index) {
  //   this.pipeline = index;
  //   this.camera.setPipelineIndex(this.pipeline);
  // }

  // @Override
  // public void updateAprilTag(VisionIOInputs inputs) {
  //   var resList = camera.getAllUnreadResults();
  //   if (!resList.isEmpty()) {
  //     PhotonPipelineResult res = resList.get(resList.size() - 1);
  //     // var res = camera.getLatestResult();

  //     // for (PhotonPipelineResult res : resList) {
  //     inputs.timestampSeconds = res.getTimestampSeconds();
  //     inputs.hasAprilTagResult = false;

  //     if (res.getMultiTagResult().isPresent()) {

  //       Transform3d fieldToCamera = res.getMultiTagResult().get().estimatedPose.best;
  //       inputs.hasAprilTagResult = true;
  //       inputs.estimatedRobotPose =
  //           new Pose3d()
  //               .plus(fieldToCamera)
  //               .relativeTo(FieldConstants.aprilTagFieldLayout.getOrigin())
  //               .plus(robotToCamera.inverse());
  //       inputs.detectedTagsIds =
  //           res.getMultiTagResult().get().fiducialIDsUsed.stream()
  //               .mapToInt(Short::intValue)
  //               .toArray();
  //       inputs.visionMeasurementStdDevs = MULTI_TARGET_STDDEVS;
  //     } else if (!res.getTargets().isEmpty()) {

  //       // Find the detected target with the lowest pose ambiguity that is within the field
  // layout
  //       double lowestAmbiguityScore = Double.POSITIVE_INFINITY;
  //       PhotonTrackedTarget lowestAmbiguityTarget = null;
  //       var tagsUsed = new ArrayList<Integer>();
  //       for (var target : res.getTargets()) {
  //         double targetPoseAmbiguity = target.getPoseAmbiguity();

  //         //   Filter out tags that aren't fiducial targets (IDK how) or in the ATFL
  //         if (targetPoseAmbiguity == -1
  //             || FieldConstants.aprilTagFieldLayout.getTagPose(target.getFiducialId()).isEmpty())
  //           continue;

  //         tagsUsed.add(target.getFiducialId());

  //         if (targetPoseAmbiguity < lowestAmbiguityScore
  //             && targetPoseAmbiguity <= AMBIGUITY_THRESHOLD) {
  //           lowestAmbiguityScore = targetPoseAmbiguity;
  //           lowestAmbiguityTarget = target;
  //         }
  //       }

  //       if (lowestAmbiguityTarget != null) {
  //         var tagPoseOpt =
  //
  // FieldConstants.aprilTagFieldLayout.getTagPose(lowestAmbiguityTarget.getFiducialId());
  //         if (tagPoseOpt.isPresent()) {
  //           inputs.hasAprilTagResult = true;
  //           inputs.estimatedRobotPose =
  //               tagPoseOpt
  //                   .get()
  //                   .transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse())
  //                   .transformBy(robotToCamera.inverse());
  //           inputs.detectedTagsIds = tagsUsed.stream().mapToInt(Integer::intValue).toArray();
  //           inputs.visionMeasurementStdDevs = SINGLE_TARGET_STDDEVS;
  //         }
  //       }
  //     }

  //     // Reject pose estimates outside reasonable bounds
  //     if (!inputs.hasAprilTagResult
  //         || inputs.estimatedRobotPose.getX() < -FIELD_BORDER_MARGIN
  //         || inputs.estimatedRobotPose.getX()
  //             > FieldConstants.aprilTagFieldLayout.getFieldLength() + FIELD_BORDER_MARGIN
  //         || inputs.estimatedRobotPose.getY() < -FIELD_BORDER_MARGIN
  //         || inputs.estimatedRobotPose.getY()
  //             > FieldConstants.aprilTagFieldLayout.getFieldWidth() + FIELD_BORDER_MARGIN
  //         || inputs.estimatedRobotPose.getZ() < -Z_MARGIN
  //         || inputs.estimatedRobotPose.getZ() > Z_MARGIN) {
  //       // Record default values if no results or invalid were found
  //       inputs.hasAprilTagResult = false;
  //       inputs.estimatedRobotPose = new Pose3d();
  //       inputs.detectedTagsIds = new int[] {};
  //       inputs.detectedTagPoses = new Pose3d[] {};
  //       inputs.visionMeasurementStdDevs = INVALID_STDDEVS;
  //     } else {
  //       // Calculate the average distance between the estimated pose and the pose of all detected
  //       // fiducial targets in three dimensions
  //       double totalDistance = 0.0;
  //       // Guaranteed to have at least one target
  //       int numTags = inputs.detectedTagsIds.length;
  //       inputs.detectedTagPoses = new Pose3d[numTags];
  //       for (int i = 0; i < numTags; i++) {
  //         int tagId = inputs.detectedTagsIds[i];
  //         // All values are guaranteed to exist within the ATFL
  //         var tagPose = FieldConstants.aprilTagFieldLayout.getTagPose(tagId).orElseThrow();
  //         inputs.detectedTagPoses[i] = tagPose;
  //         totalDistance +=
  //             tagPose.getTranslation().getDistance(inputs.estimatedRobotPose.getTranslation());
  //       }
  //       double averageDistance = totalDistance / numTags;
  //       // Increase std devs based on the average distance to the target and per camera bias
  // matrix
  //       inputs.visionMeasurementStdDevs =
  //           inputs
  //               .visionMeasurementStdDevs
  //               .times(1.0 + ((Math.pow(averageDistance, 2.0) / numTags)))
  //               .elementTimes(cameraBias);
  //     }
  //   }
  // }
}
