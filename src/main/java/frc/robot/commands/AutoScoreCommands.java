package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants.*;
import frc.robot.RobotState;
import frc.robot.subsystems.dispenser.DispenserBase;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.elevator.ElevatorBase;
import frc.robot.subsystems.elevator.ElevatorPose.CoralDispenserPose;
import frc.robot.subsystems.elevator.ElevatorPose.Preset;
import frc.robot.subsystems.intake.IntakeBase;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AutoScoreCommands {
  private static final LoggedTunableNumber maxDistanceReefLineupX =
      new LoggedTunableNumber("AutoScore/MaxDistanceReefLineupX", 0.75);
  private static final LoggedTunableNumber maxDistanceReefLineupY =
      new LoggedTunableNumber("AutoScore/MaxDistanceReefLineupY", 1.2);
  public static final LoggedTunableNumber minDistanceReefClearL4 =
      new LoggedTunableNumber("AutoScore/MinDistanceReefClear", Units.inchesToMeters(1.0));
  public static final LoggedTunableNumber minAngleReefClear =
      new LoggedTunableNumber("AutoScore/MinAngleReefClear", 30.0);
  private static final LoggedTunableNumber distanceSuperstructureReady =
      new LoggedTunableNumber("AutoScore/DistanceSuperstructureReady", 2);
  private static final LoggedTunableNumber distanceSuperstructureReadyAuto =
      new LoggedTunableNumber("AutoScore/DistanceSuperstructureReadyAuto", 2.0);

  private static final LoggedTunableNumber thetaToleranceReady =
      new LoggedTunableNumber("AutoScore/ThetaToleranceReady", 35.0);
  private static final LoggedTunableNumber arcDistanceReady =
      new LoggedTunableNumber("AutoScore/ArcDistanceReady", 0.7);
  private static final LoggedTunableNumber arcDistanceReadyAuto =
      new LoggedTunableNumber("AutoScore/ArcDistanceReadyAuto", 1.5);
  private static final LoggedTunableNumber[] linearXToleranceEject = {
    new LoggedTunableNumber("AutoScore/LinearXToleranceEject/L1", 0.03),
    new LoggedTunableNumber("AutoScore/LinearXToleranceEject/L2", 0.15),
    new LoggedTunableNumber("AutoScore/LinearXToleranceEject/L3", 0.15),
    new LoggedTunableNumber("AutoScore/LinearXToleranceEject/L4", 0.05)
  };
  private static final LoggedTunableNumber[] linearYToleranceEject = {
    new LoggedTunableNumber("AutoScore/LinearYToleranceEject/L1", 0.05),
    new LoggedTunableNumber("AutoScore/LinearYToleranceEject/L2", 0.15),
    new LoggedTunableNumber("AutoScore/LinearYToleranceEject/L3", 0.15),
    new LoggedTunableNumber("AutoScore/LinearYToleranceEject/L4", 0.08)
  };
  private static final LoggedTunableNumber[] maxLinearVel = {
    new LoggedTunableNumber("AutoScore/MaxLinearVel/L1", 0.1),
    new LoggedTunableNumber("AutoScore/MaxLinearVel/L2", 0.1),
    new LoggedTunableNumber("AutoScore/MaxLinearVel/L3", 0.1),
    new LoggedTunableNumber("AutoScore/MaxLinearVel/L4", 0.05)
  };
  private static final LoggedTunableNumber[] maxAngularVel = {
    new LoggedTunableNumber("AutoScore/MaxAngularVel/L1", 10),
    new LoggedTunableNumber("AutoScore/MaxAngularVel/L2", 10),
    new LoggedTunableNumber("AutoScore/MaxAngularVel/L3", 10),
    new LoggedTunableNumber("AutoScore/MaxAngularVel/L4", 10)
  };
  private static final LoggedTunableNumber[] thetaToleranceEject = {
    new LoggedTunableNumber("AutoScore/ThetaToleranceEject/L1", 5),
    new LoggedTunableNumber("AutoScore/ThetaToleranceEject/L2", 5),
    new LoggedTunableNumber("AutoScore/ThetaToleranceEject/L3", 5),
    new LoggedTunableNumber("AutoScore/ThetaToleranceEject/L4", 10)
  };
  private static final LoggedTunableNumber l4EjectDelay =
      new LoggedTunableNumber("AutoScore/L4EjectDelay", 0.05);
  private static final LoggedTunableNumber l4EjectDelayAuto =
      new LoggedTunableNumber("AutoScore/L4EjectDelayAuto", 0.05);
  private static final LoggedTunableNumber l1AlignOffsetX =
      new LoggedTunableNumber("AutoScore/L1AlignOffsetX", 0.45);
  private static final LoggedTunableNumber l1AlignOffsetY =
      new LoggedTunableNumber("AutoScore/L1AlignOffsetY", 0.0);
  private static final LoggedTunableNumber l1AlignOffsetTheta =
      new LoggedTunableNumber("AutoScore/L1AlignOffsetTheta", 180.0);
  private static final LoggedTunableNumber minDistanceAim =
      new LoggedTunableNumber("AutoScore/MinDistanceAim", 0.2);
  private static final LoggedTunableNumber[] ejectTimeSeconds = {
    new LoggedTunableNumber("AutoScore/EjectTimeSeconds/L1", 0.5),
    new LoggedTunableNumber("AutoScore/EjectTimeSeconds/L2", 0.5),
    new LoggedTunableNumber("AutoScore/EjectTimeSeconds/L3", 0.5),
    new LoggedTunableNumber("AutoScore/EjectTimeSeconds/L4", 0.3)
  };
  private static final LoggedTunableNumber correctiveMeasureFF =
      new LoggedTunableNumber("AutoScore/CorrectiveMeasureFF", 0.7);
  private static final LoggedTunableNumber correctiveMeasureDistance =
      new LoggedTunableNumber("AutoScore/CorrectiveMeasureDistance", Units.inchesToMeters(20.0));
  private static final LoggedTunableNumber correctiveMeasureBlendDistance =
      new LoggedTunableNumber(
          "AutoScore/CorrectiveMeasureBlendDistance", Units.inchesToMeters(6.0));
  private static final LoggedTunableNumber maxAimingAngle =
      new LoggedTunableNumber("AutoScore/MaxAimingAngle", 5.0);
  private static final LoggedTunableNumber[] branchFudgeX = {
    new LoggedTunableNumber("AutoScore/FudgeX/L1", 0.0),
    new LoggedTunableNumber("AutoScore/FudgeX/L2", 0.0),
    new LoggedTunableNumber("AutoScore/FudgeX/L3", 0.0),
    new LoggedTunableNumber("AutoScore/FudgeX/L4", 0.0)
  };

  // public static Command autoAlign(
  //     DriveBase driveBase, ElevatorBase elevatorBase, Supplier<CoralObjective> coralObjective) {
  //   return autoAlign(driveBase, elevatorBase, coralObjective, () -> 0, () -> 0, () -> 0);
  // }
  //
  // public static Command autoAlign(
  //     DriveBase driveBase,
  //     ElevatorBase elevatorBase,
  //     Supplier<CoralObjective> coralObjective,
  //     DoubleSupplier driverX,
  //     DoubleSupplier driverY,
  //     DoubleSupplier driverOmega) {
  //   Supplier<Pose2d> robot = () -> AutoScoreCommands.getRobotPose(coralObjective.get());
  //
  //   Function<CoralObjective, Pose2d> goal =
  //       objective -> {
  //         if (objective.reefLevel == ReefLevel.L1) {
  //           return AllianceFlipUtil.apply(getL1Pose(objective));
  //         }
  //
  //         Pose2d goalPose = getCoralScorePose(objective);
  //         final double clearDistance = minDistanceReefClearL4.get();
  //         boolean isL4 = objective.reefLevel == ReefLevel.L4;
  //
  //         if ((!elevatorBase.readyForL4() && isL4)
  //             || (elevatorBase.readyForL4()
  //                 && withinDistanceToReef(robot.get(), clearDistance - 0.05)
  //                 && !isL4)) {
  //           goalPose =
  //               goalPose.transformBy(
  //                   GeomUtil.toTransform2d(
  //                       goalPose.getTranslation().getDistance(Reef.center)
  //                           - Reef.faceLength
  //                           - (DriveConstants.robotWidth / 2.0)
  //                           - clearDistance,
  //                       0.0));
  //         }
  //         Logger.recordOutput("AutoScore/Goal", AllianceFlipUtil.apply(goalPose));
  //
  //         if (DriverStation.isAutonomousEnabled()) {
  //           return AllianceFlipUtil.apply(goalPose);
  //         } else {
  //           Pose2d flippedGoalPose = AllianceFlipUtil.apply(goalPose);
  //           Rotation2d originalRotation = flippedGoalPose.getRotation();
  //           Rotation2d rotationAdjustment =
  //               AllianceFlipUtil.apply(getBranchPose(objective))
  //                   .getTranslation()
  //                   .minus(robot.get().getTranslation())
  //                   .getAngle()
  //                   .minus(originalRotation);
  //           rotationAdjustment =
  //               Rotation2d.fromDegrees(
  //                   MathUtil.clamp(
  //                       rotationAdjustment.getDegrees(),
  //                       -maxAimingAngle.get(),
  //                       maxAimingAngle.get()));
  //           return new Pose2d(
  //               flippedGoalPose.getTranslation(), originalRotation.plus(rotationAdjustment));
  //         }
  //       };
  //
  //   return new DriveToPose(
  //       driveBase,
  //       () -> getDriveTarget(robot.get(), goal.apply(coralObjective.get())),
  //       robot,
  //       () ->
  //           DriveCommands.getLinearVelocityFromJoysticks(
  //                   driverX.getAsDouble(), driverY.getAsDouble())
  //               .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
  //       () -> DriveCommands.getOmegaFromJoysticks(driverOmega.getAsDouble()));
  // }

  public static Command autoScore(
      DriveBase driveBase,
      ElevatorBase elevatorBase,
      DispenserBase dispenserBase,
      IntakeBase intakeBase,
      Supplier<ReefLevel> reefLevel,
      Supplier<Optional<CoralObjective>> coralObjective,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega,
      Command joystickDrive,
      BooleanSupplier robotRelative,
      BooleanSupplier disableReefAutoAlign,
      BooleanSupplier manualEject) {
    Supplier<Pose2d> robot =
        () ->
            coralObjective
                .get()
                .map(AutoScoreCommands::getRobotPose)
                .orElseGet(RobotState.getInstance()::getEstimatedPose);

    //   Function<CoralObjective, Pose2d> goal =
    //       objective -> {
    //         if (objective.reefLevel == ReefLevel.L1) {
    //           return AllianceFlipUtil.apply(getL1Pose(objective));
    //         }
    //
    //         Pose2d goalPose = getCoralScorePose(objective);
    //         final double clearDistance = minDistanceReefClearL4.get();
    //         boolean isL4 = objective.reefLevel == ReefLevel.L4;
    //
    //         if ((!elevatorBase.readyForL4() && isL4)
    //             || (elevatorBase.readyForL4()
    //                 && withinDistanceToReef(robot.get(), clearDistance - 0.05)
    //                 && !isL4)) {
    //           goalPose =
    //               goalPose.transformBy(
    //                   GeomUtil.toTransform2d(
    //                       goalPose.getTranslation().getDistance(Reef.center)
    //                           - Reef.faceLength
    //                           - (DriveConstants.robotWidth / 2.0)
    //                           - clearDistance,
    //                       0.0));
    //         }
    //         Logger.recordOutput("AutoScore/Goal", AllianceFlipUtil.apply(goalPose));
    //
    //         if (DriverStation.isAutonomousEnabled()) {
    //           return AllianceFlipUtil.apply(goalPose);
    //         } else {
    //           Pose2d flippedGoalPose = AllianceFlipUtil.apply(goalPose);
    //           Rotation2d originalRotation = flippedGoalPose.getRotation();
    //           Rotation2d rotationAdjustment =
    //               AllianceFlipUtil.apply(getBranchPose(objective))
    //                   .getTranslation()
    //                   .minus(robot.get().getTranslation())
    //                   .getAngle()
    //                   .minus(originalRotation);
    //           rotationAdjustment =
    //               Rotation2d.fromDegrees(
    //                   MathUtil.clamp(
    //                       rotationAdjustment.getDegrees(),
    //                       -maxAimingAngle.get(),
    //                       maxAimingAngle.get()));
    //           return new Pose2d(
    //               flippedGoalPose.getTranslation(), originalRotation.plus(rotationAdjustment));
    //         }
    //       };
    //
    //   var driveCommand =
    //       new DriveToPose(
    //           driveBase,
    //           () ->
    //               coralObjective
    //                   .get()
    //                   .map(objective -> getDriveTarget(robot.get(), goal.apply(objective)))
    //                   .orElseGet(RobotState.getInstance()::getEstimatedPose),
    //           robot,
    //           () ->
    //               DriveCommands.getLinearVelocityFromJoysticks(
    //                       driverX.getAsDouble(), driverY.getAsDouble())
    //                   .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
    //           () -> DriveCommands.getOmegaFromJoysticks(driverOmega.getAsDouble()));
    //
    //   var needsToGetBack = new AtomicBoolean(false);
    //   var hasEnded = new AtomicBoolean(false);
    //
    //   // Schedule get back command
    //   new Trigger(() -> hasEnded.get() && needsToGetBack.get())
    //       .and(() -> !disableReefAutoAlign.getAsBoolean())
    //       .onTrue(
    //           getBackCorrectiveMeasure(driveBase, driverX, driverY, driverOmega, robotRelative)
    //               .finallyDo(() -> needsToGetBack.set(false))
    //               .withName("Corrective Measure"));
    //
    //   return Commands.runOnce(
    //           () -> {
    //             // Start LEDs
    //             LEDBase.getInstance().autoScoringReef = true;
    //
    //             needsToGetBack.set(false);
    //             hasEnded.set(false);
    //           })
    //       .finallyDo(
    //           interrupted -> {
    //             // Stop LEDs
    //             LEDBase.getInstance().autoScoringReef = false;
    //
    //             // Indicate has ended command
    //             hasEnded.set(true);
    //           });

    return Commands.none();
  }

  public static Command autoScore(
      DriveBase driveBase,
      ElevatorBase elevatorBase,
      DispenserBase dispenserBase,
      IntakeBase intakeBase,
      Supplier<ReefLevel> reefLevel,
      Supplier<Optional<CoralObjective>> coralObjective) {
    return autoScore(
        driveBase,
        elevatorBase,
        dispenserBase,
        intakeBase,
        reefLevel,
        coralObjective,
        () -> 0,
        () -> 0,
        () -> 0,
        Commands.none(),
        () -> false,
        () -> false,
        () -> false);
  }

  private static Command aimAndEject(
      ElevatorBase elevatorBase,
      DispenserBase dispenserBase,
      Supplier<ReefLevel> reefLevel,
      BooleanSupplier eject,
      BooleanSupplier holdEject) {
    final Timer ejectTimer = new Timer();
    return elevatorBase
        .runGoal(() -> Preset.fromLevel(reefLevel.get()))
        .until(eject)
        .andThen(
            Commands.runOnce(ejectTimer::restart),
            elevatorBase
                .runGoal(() -> Preset.fromLevel(reefLevel.get()))
                .alongWith(
                    Commands.waitUntil(elevatorBase::atGoal)
                        .andThen(
                            dispenserBase
                                .runDispenser(
                                    () ->
                                        dispenserBase.getDispenserVoltageFromLevel(reefLevel.get()))
                                .until(
                                    () ->
                                        ejectTimer.hasElapsed(
                                                ejectTimeSeconds[reefLevel.get().ordinal()].get())
                                            && !holdEject.getAsBoolean()))));
  }

  private static Command preIntake(
      ElevatorBase elevator,
      IntakeBase intake,
      DispenserBase dispenser,
      Supplier<Pose2d> robot,
      BooleanSupplier shouldClearReef,
      BooleanSupplier disableReefAutoAlign) {
    return Commands.waitUntil(
            () ->
                outOfDistanceToReef(robot.get(), minDistanceReefClearL4.get())
                    || !shouldClearReef.getAsBoolean()
                    || disableReefAutoAlign.getAsBoolean())
        .andThen(IntakeCommands.intake(elevator, intake, dispenser))
        .onlyIf(() -> !dispenser.holdingCoral());
  }

  private static Command getBackCorrectiveMeasure(
      DriveBase driveBase,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega,
      BooleanSupplier robotRelative) {
    Supplier<Translation2d> correctiveMeasure =
        () -> {
          var flippedRobot = AllianceFlipUtil.apply(RobotState.getInstance().getEstimatedPose());
          Translation2d driverLinearVel =
              DriveCommands.getLinearVelocityFromJoysticks(
                  driverX.getAsDouble(), driverY.getAsDouble());

          return driverLinearVel.plus(
              flippedRobot
                  .getTranslation()
                  .minus(Reef.center)
                  .div(flippedRobot.getTranslation().getDistance(Reef.center))
                  .times(
                      correctiveMeasureFF.get()
                          * (1.0
                              - MathUtil.clamp(
                                  (flippedRobot.getTranslation().getDistance(Reef.center)
                                          - Reef.faceLength
                                          - DriveConstants.robotWidth / 2.0
                                          - correctiveMeasureBlendDistance.get())
                                      / (correctiveMeasureDistance.get()
                                          - correctiveMeasureBlendDistance.get()),
                                  0.0,
                                  1.0)))
                  .times(driverLinearVel.getNorm()));
        };

    return DriveCommands.joystickDrive(
            driveBase,
            () -> correctiveMeasure.get().getX(),
            () -> correctiveMeasure.get().getY(),
            driverOmega,
            robotRelative)
        .until(
            () ->
                outOfDistanceToReef(
                    RobotState.getInstance().getEstimatedPose(), correctiveMeasureDistance.get()));
  }

  private static Pose2d getDriveTarget(Pose2d robot, Pose2d goal) {
    Rotation2d angleToGoal =
        robot
            .getTranslation()
            .minus(AllianceFlipUtil.apply(Reef.center))
            .getAngle()
            .minus(goal.getTranslation().minus(AllianceFlipUtil.apply(Reef.center)).getAngle());
    Logger.recordOutput("AutoScore/AngleToGoal", angleToGoal);
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
            -shiftXT * maxDistanceReefLineupX.get(),
            Math.copySign(shiftYT * maxDistanceReefLineupY.get(), offset.getY())));
  }

  public static boolean readyForScore(Pose2d robot, Pose2d goal, boolean shouldBackUp) {
    double arcDistance =
        robot
                .getTranslation()
                .minus(AllianceFlipUtil.apply(Reef.center))
                .getAngle()
                .minus(goal.getTranslation().minus(AllianceFlipUtil.apply(Reef.center)).getAngle())
                .getRadians()
            * robot.getTranslation().getDistance(AllianceFlipUtil.apply(Reef.center));
    return withinDistanceToReef(
            robot,
            DriverStation.isAutonomousEnabled()
                ? distanceSuperstructureReadyAuto.get()
                : distanceSuperstructureReady.get())
        && (outOfDistanceToReef(robot, minDistanceReefClearL4.get() - 0.1) || !shouldBackUp)
        && Math.abs(robot.relativeTo(goal).getRotation().getDegrees()) <= thetaToleranceReady.get()
        && Math.abs(arcDistance)
            <= (DriverStation.isAutonomousEnabled()
                ? arcDistanceReadyAuto.get()
                : arcDistanceReady.get());
  }

  public static boolean withinDistanceToReef(Pose2d robot, double distance) {
    final double distanceToReefCenter =
        AllianceFlipUtil.apply(robot).getTranslation().getDistance(Reef.center);
    Logger.recordOutput("AutoScore/DistanceToReefCenter", distanceToReefCenter);
    return distanceToReefCenter <= Reef.faceLength + DriveConstants.robotWidth / 2.0 + distance;
  }

  public static boolean outOfDistanceToReef(Pose2d robot, double distance) {
    final double distanceToReefCenter =
        AllianceFlipUtil.apply(robot).getTranslation().getDistance(Reef.center);
    Logger.recordOutput("AutoScore/DistanceToReefCenter", distanceToReefCenter);
    return distanceToReefCenter >= Reef.faceLength + DriveConstants.robotWidth / 2.0 + distance;
  }

  private static Pose2d getRobotPose(CoralObjective coralObjective) {
    return RobotState.getInstance()
        .getReefPose(coralObjective.branchId() / 2, getCoralScorePose(coralObjective));
  }

  private static Pose2d getL1Pose(CoralObjective coralObjective) {
    return Reef.centerFaces[coralObjective.branchId() / 2].transformBy(
        new Transform2d(
            l1AlignOffsetX.get(),
            l1AlignOffsetY.get(),
            Rotation2d.fromDegrees(l1AlignOffsetTheta.get())));
  }

  /** Get position of robot aligned with branch for provided objective. */
  private static Pose2d getCoralScorePose(CoralObjective coralObjective) {
    return getBranchPose(coralObjective)
        .transformBy(getCoralDispenserPose(coralObjective.reefLevel()).toRobotPose());
  }

  private static Pose2d getBranchPose(CoralObjective objective) {
    return Reef.branchPositions2d
        .get(objective.branchId())
        .get(objective.reefLevel())
        .transformBy(
            new Transform2d(
                branchFudgeX[objective.reefLevel().ordinal()].get(), 0, Rotation2d.kZero));
  }

  private static CoralDispenserPose getCoralDispenserPose(ReefLevel reefLevel) {
    return switch (reefLevel) {
      case L1, L2 -> CoralDispenserPose.L2_CORAL;
      case L3 -> CoralDispenserPose.L3_CORAL;
      case L4 -> CoralDispenserPose.L4_CORAL;
    };
  }

  public record CoralObjective(int branchId, ReefLevel reefLevel) {}
}
