package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.PoseEstimator;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.List;

public class AutoScoreCommands {
  private static final LoggedTunableNumber l4ScoringOffset =
      new LoggedTunableNumber("AutoScore/l4ScoringOffset", Units.inchesToMeters(4.5));

  public enum ReefSide {
    LEFT,
    RIGHT
  }

  public Command alignToReef(DriveBase driveBase, ReefSide reefSide) {
    return new DriveToPose(
        driveBase,
        () ->
            PoseEstimator.getInstance()
                .getEstimatedPose()
                .nearest(List.of(FieldConstants.Reef.centerFaces))
                .transformBy(
                    reefSide == ReefSide.LEFT
                        ? FieldConstants.Reef.centerToLeftBranch
                        : FieldConstants.Reef.centerToRightBranch));
  }
}
