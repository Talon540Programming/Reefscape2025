package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

  private final VisionIO[] cameras;
  private final VisionIOInputs[] cameraInputs;
  public final double rightPostCenterXAligned = 425;
  public final double leftBranchCenterXAligned = 1134;

  public Vision(VisionIO... cameras) {
    this.cameras = cameras;
    cameraInputs = new VisionIOInputs[this.cameras.length];
    for (int i = 0; i < this.cameras.length; i++) {
      cameraInputs[i] = new VisionIOInputs();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < cameras.length; i++) {
      var input = cameraInputs[i];

      cameras[i].updateInputs(input);
      Logger.processInputs("Vision/Cam" + i, input);

      // Don't report if there is no valid global pose estimate

      // if (input.pipelineIndex == 0) {
      // PoseEstimator.getInstance()
      //     .addVisionObservation(
      //         new VisionObservation(
      //             input.estimatedRobotPose.toPose2d(),
      //             input.timestampSeconds,
      //             input.visionMeasurementStdDevs));
      // } else if (input.pipelineIndex == 1) {
      // PoseEstimator.getInstance()
      //     .addTxTyObservation(
      //         new TxTyObservation(
      //             input.detectedTagsIds[0],
      //             cameras[i].getRobotToCamera(),
      //             input.detectedCorners,
      //             input.tagDistance,
      //             input.timestampSeconds));
      // }

      // Logger.recordOutput("Vision/offset", getOffsetX(0));
    }
  }

  // public void setPipelineIndex(int index) {
  //   for (VisionIO camera : cameras) {
  //     camera.setPipelineIndex(index);
  //   }
  // }

  // Shelby County v Holder
  // Griswald v Connecticut
  // Lawrence v Texas
  // Loving v Virginia
  // Hernandez v Texas
  // Reed v Reed
  // Obergefell v Hodges
  // Plyler v Doe
  // Korematsu v United States
  // Shah v Reno

  // public Pose2d getNearestReefPose() {
  //   Pose2d currentPose = PoseEstimator.getInstance().getReefPose();
  // }

  // public Pose2d getNearestReefFacePose() {
  //   Pose2d currentPose = PoseEstimator.getInstance().getEstimatedPose();

  //   Pose2d nearestFace =
  //       currentPose.nearest(
  //           Arrays.asList(FieldConstants.Reef.centerFaces).stream()
  //               .map(pose -> AllianceFlipUtil.apply(pose))
  //               .toList());

  //   return nearestFace;
  // }

  public double getOffsetX(int cameraindex) {
    // for (int i = 0; i < cameras.length; i++) {
    // int i = isLeftPole ? 0 : 1;
    var input = cameraInputs[cameraindex];

    if (!input.hasResult) return 0;

    double centerx = input.tagCenterX;

    return centerx;

    // if (cameraindex == 0) {
    //   return centerx;
    // } else if (cameraindex == 1) {
    //   return centerx;
    // } else return Integer.MAX_VALUE;

    // return cameraindex == 0
    //     ? centerx - leftCameraCenterXAligned
    //     : centerx - rightCameraCenterXAlined;
  }

  // public double getHorizontalDistanceToTag() {

  // }

  // public Pose2d getNearestReefPose() {
  //   Pose2d currentPose = PoseEstimator.getInstance().getEstimatedPose();
  //   // Pose2d closestPose = Pose2d.kZero;
  //   // for (Pose2d pose : FieldConstants.Reef.centerFaces) {

  //   Pose2d closestPose =
  //       currentPose.nearest(
  //           Arrays.asList(
  //               FieldConstants.Reef.branchPositions2d.stream()
  //                   .map(map -> map.get(FieldConstants.ReefLevel.L2))
  //                   .toArray(Pose2d[]::new)));

  //   return closestPose;

  // int faceNum = Arrays.binarySearch(FieldConstants.Reef.centerFaces, closestPose);

  // return AllianceFlipUtil.shouldFlip() ? redAllianceAprilTagReefPoses.get(faceNum) :
  // blueAllianceAprilTagReefPoses.get(faceNum);
  // }

  // public int getReefAprilTagId() {
  //   int closestFace = getNearestReefSide();
  //   // System.out.println(closestFace);

  //   return VisionConstants.redAllianceAprilTagReefPoses[closestFace];
  // }

  // public int getNearestReefSide() {
  //   Pose2d closestPose = getNearestReefPose();
  //   int closestFace;
  //   for (int i = 0; i < 6; i++) {
  //     // if (FieldConstants.Reef.centerFaces[i].equals(closestPose)) {
  //     //   closestFace = i;
  //     //   return closestFace;
  //     // }
  //     if (FieldConstants.Reef.branchPositions2d
  //         .get(i)
  //         .get(FieldConstants.ReefLevel.L2)
  //         .equals(closestPose)) {
  //       closestFace = i;
  //       return closestFace;
  //     }
  //   }
  //   // int closestFace = Arrays.binarySearch(FieldConstants.Reef.centerFaces, closestPose);
  //   return -1;
  // }
}
