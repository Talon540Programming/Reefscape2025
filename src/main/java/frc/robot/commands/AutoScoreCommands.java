package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;
import frc.robot.PoseEstimator;
import frc.robot.util.LoggedTunableNumber;
import java.util.Arrays;
import org.littletonrobotics.junction.AutoLogOutput;

public class AutoScoreCommands {
  private static final LoggedTunableNumber coralScoringOffset =
      new LoggedTunableNumber(
          "AutoScore/CoralScoringOffsetMeters", Units.inchesToMeters(-(26.0 + 3.625) / 2.0) - 4.0);

  public static Pose2d getNearestReefFace() {
    return PoseEstimator.getInstance()
        .getEstimatedPose()
        .nearest(Arrays.asList(FieldConstants.Reef.centerFaces));

    // for (int i = 0; i < cameras.length; i++) {
    //   var input = cameraInputs[i];
    //   if (input.detectedTagsIds.length == 0) continue;
    //   for (int j = 0; j < input.detectedTagsIds.length; j++) {
    //     for (int k = 0; k < VisionConstants.reefAprilTags.length; k++) {
    //       if (input.detectedTagsIds[j] == VisionConstants.reefAprilTags[k]
    //           && input.tagDistances[j] <= maxReefTagDistance) {
    //         // Logger.recordOutput("Vision/ReefFacePose", FieldConstants.Reef.centerFaces[k]);
    //         return FieldConstants.Reef.centerFaces[k];
    //       }
    //       ;
    //     }
    //   }
    // }

    // return PoseEstimator.getInstance().getEstimatedPose();
  }

  @AutoLogOutput(key = "Vision/NearestLeftBranch")
  public static Pose2d getNearestLeftBranch() {
    return getNearestReefFace().plus(FieldConstants.Reef.centerToLeftBranch);
  }

  @AutoLogOutput(key = "Vision/NearestRightBranch")
  public static Pose2d getNearestRightBranch() {
    return getNearestReefFace().plus(FieldConstants.Reef.centerToRightBranch);
  }
}
