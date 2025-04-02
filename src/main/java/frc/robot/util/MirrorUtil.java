package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.FieldConstants;
import frc.robot.commands.AutoScoreCommands.CoralObjective;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;

public class MirrorUtil {
  @Setter @Getter private static BooleanSupplier mirror;

  public static CoralObjective apply(CoralObjective coralObjective) {
    if (!mirror.getAsBoolean()) return coralObjective;
    int shiftedBranchId =
        (coralObjective.branchId() + 11) % 12; // Shift left by 1 (handling 0 -> 11)
    int flippedBranchId = (12 - shiftedBranchId) % 12; // Flip and wrap around
    return new CoralObjective(flippedBranchId, coralObjective.reefLevel());
  }

  public static Pose2d apply(Pose2d pose) {
    if (!mirror.getAsBoolean()) return pose;
    return new Pose2d(
        pose.getX(),
        FieldConstants.fieldWidth - pose.getY(),
        new Rotation2d(pose.getRotation().getCos(), -pose.getRotation().getSin()));
  }
}
