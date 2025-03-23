package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Reef;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;

public class AutoScoreCommands {
  private static final LoggedTunableNumber coralScoringOffset =
      new LoggedTunableNumber(
          "AutoScore/CoralScoringOffsetMeters", Units.inchesToMeters(-(26.0 + 3.625) / 2.0) - 4.0);
  private static final LoggedTunableNumber correctiveMeasureDistance =
      new LoggedTunableNumber("AutoScore/CorrectiveMeasureOffset", Units.inchesToMeters(-2.0));

  // public static Command autoScore(
  //     DriveBase driveBase,
  //     ElevatorBase elevatorBase,
  //     DispenserBase dispenserBase,
  //     Supplier<CoralObjective> objective) {
  //   return Commands.sequence(
  //           autoAlign(driveBase, objective),
  //           Commands.deadline(
  //               Commands.waitUntil(elevatorBase::isAtGoal),
  //               Commands.run(
  //                   () ->
  //                       elevatorBase.setGoal(
  //                           ElevatorState.getCoralScoringState(objective.get().reefLevel())))),
  //           dispenserBase.eject(elevatorBase::getGoal),
  //           getBackCorrectiveMeasure(driveBase, objective)
  //               .onlyIf(() -> objective.get().reefLevel == FieldConstants.ReefLevel.L4),
  //           Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.STOW)))
  //       .beforeStarting(() -> LEDBase.getInstance().autoScoringReef = true)
  //       .finallyDo(() -> LEDBase.getInstance().autoScoringReef = false);
  // }

  public static Command autoAlign(
      DriveBase driveBase, Supplier<Pose2d> pose, Supplier<Integer> branch) {
    var alignCommand =
        new DriveToPose(driveBase, () -> getCoralScorePose(branch.get(), pose.get()));
    return alignCommand.until(alignCommand::atGoal);
  }

  // public static Command autoAlign(DriveBase driveBase, Supplier<CoralObjective> objective) {
  //   var alignCommand = new DriveToPose(driveBase, () -> getCoralScorePose(objective.get()));
  //   return alignCommand.until(alignCommand::atGoal);
  // }

  //   private static Command getBackCorrectiveMeasure(DriveBase driveBase, Supplier<CoralObjective>
  // objective) {
  //     var moveBackCommand = new DriveToPose(driveBase, () ->
  // getCoralScorePose(objective.get()).transformBy(new Transform2d(correctiveMeasureDistance.get(),
  // 0, new Rotation2d())));
  //     return moveBackCommand.until(moveBackCommand::atGoal);
  //   }

  private static Command getBackCorrectiveMeasure(
      DriveBase driveBase,
      Supplier<Integer> branch,
      Supplier<Pose2d> pose,
      Supplier<FieldConstants.ReefLevel> reefLevel) {
    var moveBackCommand =
        new DriveToPose(
            driveBase,
            () ->
                getCoralScorePose(branch.get(), pose.get())
                    .transformBy(
                        new Transform2d(correctiveMeasureDistance.get(), 0, new Rotation2d())));
    return moveBackCommand.until(moveBackCommand::atGoal);
  }

  public static Pose2d getCoralScorePose(int branch, Pose2d pose) {
    double offset = coralScoringOffset.get();
    return getBranchPose(branch, pose.transformBy(new Transform2d(offset, 0, new Rotation2d())));
  }

  public static Pose2d getBranchPose(int branch, Pose2d pose) {
    return branch == 1 ? pose.plus(Reef.centerToRightBranch) : pose.plus(Reef.centerToLeftBranch);
  }

  public record CoralObjective(int branchId, FieldConstants.ReefLevel reefLevel) {}
}
