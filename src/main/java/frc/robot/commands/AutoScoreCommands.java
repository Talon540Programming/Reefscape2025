package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.*;
import frc.robot.subsystems.dispenser.DispenserBase;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.elevator.ElevatorBase;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AutoScoreCommands {
  private static final LoggedTunableNumber maxDistanceReefLineup =
      new LoggedTunableNumber("AutoScore/MaxDistanceReefLineup", 1.5);

  private static final LoggedTunableNumber correctiveMeasureFF =
      new LoggedTunableNumber("AutoScore/CorrectiveMeasureFF", 0.7);
  private static final LoggedTunableNumber correctiveMeasureDistance =
      new LoggedTunableNumber("AutoScore/CorrectiveMeasureDistance", Units.inchesToMeters(20.0));
  private static final LoggedTunableNumber correctiveMeasureBlendDistance =
      new LoggedTunableNumber(
          "AutoScore/CorrectiveMeasureBlendDistance", Units.inchesToMeters(12.0));

  public static Command autoScore(
      DriveBase driveBase,
      ElevatorBase elevatorBase,
      DispenserBase dispenserBase,
      Supplier<Optional<CoralObjective>> objective,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega,
      Command joystickDrive,
      BooleanSupplier robotRelative,
      BooleanSupplier disableReefAutoAlign,
      BooleanSupplier manualEject) {
    return Commands.none();
  }

  public static Command autoScore(
      DriveBase driveBase,
      ElevatorBase elevatorBase,
      DispenserBase dispenserBase,
      Supplier<Optional<CoralObjective>> coralObjective) {
    return autoScore(
        driveBase,
        elevatorBase,
        dispenserBase,
        coralObjective,
        () -> 0,
        () -> 0,
        () -> 0,
        Commands.none(),
        () -> false,
        () -> false,
        () -> false);
  }

  private static Command getBackCorrectiveMeasure(
      DriveBase driveBase,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega,
      BooleanSupplier robotRelative) {
    return Commands.none();
  }

  private static Pose2d getDriveTarget(Pose2d robot, Pose2d goal) {
    Rotation2d angleToGoal =
        robot
            .getTranslation()
            .minus(AllianceFlipUtil.apply(Reef.center))
            .getAngle()
            .minus(goal.getTranslation().minus(AllianceFlipUtil.apply(Reef.center)).getAngle());

    Logger.recordOutput("AutoScore/AngleToGoal", angleToGoal);
    if (Math.abs(angleToGoal.getDegrees()) >= 30.0 && withinDistanceToReef(robot, 1)) {
      var offset = robot.relativeTo(goal);
      double yDistance = Math.abs(offset.getY());
      double xDistance = Math.abs(offset.getX());
      double shiftXT =
          MathUtil.clamp(
              (yDistance / (Reef.faceLength * 2)) + ((xDistance - 0.3) / (Reef.faceLength * 4)),
              0.0,
              1.0);
      double shiftYT =
          MathUtil.clamp(yDistance <= 0.2 ? 0.0 : offset.getX() / Reef.faceLength, 0.0, 1.0);
      return goal.transformBy(
          GeomUtil.toTransform2d(
              -shiftXT * maxDistanceReefLineup.get(),
              Math.copySign(shiftYT * maxDistanceReefLineup.get() * 0.8, offset.getY())));
    }
    return goal.transformBy(
        GeomUtil.toTransform2d(
            -MathUtil.clamp(
                    (robot.getTranslation().getDistance(goal.getTranslation()) - 1.0) / 1.2,
                    0.0,
                    1.0)
                * 0.7,
            0.0));
  }

  private static boolean withinDistanceToReef(Pose2d robot, double distance) {
    final double distanceToReefCenter =
        AllianceFlipUtil.apply(robot).getTranslation().getDistance(Reef.center);
    Logger.recordOutput("AutoScore/DistanceToReefCenter", distanceToReefCenter);
    return distanceToReefCenter <= Reef.faceLength + DriveConstants.robotWidth / 2.0 + distance;
  }

  private static boolean outOfDistanceToReef(Pose2d robot, double distance) {
    final double distanceToReefCenter =
        AllianceFlipUtil.apply(robot).getTranslation().getDistance(Reef.center);
    Logger.recordOutput("AutoScore/DistanceToReefCenter", distanceToReefCenter);
    return distanceToReefCenter >= Reef.faceLength + DriveConstants.robotWidth / 2.0 + distance;
  }

  public record CoralObjective(int branchId, ReefLevel reefLevel) {}
}
