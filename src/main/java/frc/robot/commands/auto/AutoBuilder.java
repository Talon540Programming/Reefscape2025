package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.PoseEstimator;
import frc.robot.subsystems.dispenser.DispenserBase;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.elevator.ElevatorBase;
import frc.robot.subsystems.intake.IntakeBase;
import frc.robot.util.AllianceFlipUtil;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class AutoBuilder {
  private final DriveBase driveBase;
  private final ElevatorBase elevatorBase;
  private final DispenserBase dispenserBase;
  private final IntakeBase intakeBase;

  public Command taxi() {
    return Commands.runEnd(
            () -> driveBase.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0)),
            driveBase::stop,
            driveBase)
        .withTimeout(2.0)
        .beforeStarting(
            Commands.runOnce(
                () ->
                    PoseEstimator.getInstance()
                        .resetPose(
                            new Pose2d(
                                PoseEstimator.getInstance().getEstimatedPose().getTranslation(),
                                AllianceFlipUtil.apply(Rotation2d.kPi))),
                driveBase));
  }

  public Command deadreckonedL2() {
    return Commands.sequence();
  }

  public Command deadreckonedL2Center() {
    return Commands.none();
  }

  public Command deadreckonedL4() {
    return Commands.none();
  }

  public Command deadreckonedL4Center() {
    return Commands.none();
  }

  // Call at start of auto to determine if we are starting top vs bottom
  private static boolean mirrorStartingSide() {
    return PoseEstimator.getInstance().getEstimatedPose().getMeasureY().in(Units.Meters)
        > FieldConstants.fieldWidth / 2.0;
  }
}
