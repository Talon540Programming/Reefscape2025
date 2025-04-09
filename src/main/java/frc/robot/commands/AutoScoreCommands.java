package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import frc.robot.subsystems.leds.LEDBase;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
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
    new LoggedTunableNumber("AutoScore/LinearYToleranceEject/L4", 0.05)
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
    new LoggedTunableNumber("AutoScore/ThetaToleranceEject/L4", 5)
  };
  private static final LoggedTunableNumber l4EjectDelay =
      new LoggedTunableNumber("AutoScore/L4EjectDelay", 0.05);
  private static final LoggedTunableNumber l4EjectDelayAuto =
      new LoggedTunableNumber("AutoScore/L4EjectDelayAuto", 0.05);
  private static final LoggedTunableNumber[] ejectTimeSeconds = {
    new LoggedTunableNumber("AutoScore/EjectTimeSeconds/L1", 0.5),
    new LoggedTunableNumber("AutoScore/EjectTimeSeconds/L2", 0.5),
    new LoggedTunableNumber("AutoScore/EjectTimeSeconds/L3", 0.5),
    new LoggedTunableNumber("AutoScore/EjectTimeSeconds/L4", 0.3)
  };
  private static final LoggedTunableNumber maxAimingAngle =
      new LoggedTunableNumber("AutoScore/MaxAimingAngle", 5.0);
  private static final LoggedTunableNumber[] branchFudgeX = {
    new LoggedTunableNumber("AutoScore/FudgeX/L1", 0.0),
    new LoggedTunableNumber("AutoScore/FudgeX/L2", 0.0),
    new LoggedTunableNumber("AutoScore/FudgeX/L3", 0.0),
    new LoggedTunableNumber("AutoScore/FudgeX/L4", 0.0)
  };

  public static Command autoScore(
      DriveBase driveBase,
      ElevatorBase elevatorBase,
      DispenserBase dispenserBase,
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

    Function<CoralObjective, Pose2d> goal =
        objective -> {
          if (objective.reefLevel() == ReefLevel.L1) {
            return AllianceFlipUtil.apply(getL1Pose(objective));
          }

          Pose2d goalPose = getCoralScorePose(objective);
          Logger.recordOutput("AutoScore/Goal", AllianceFlipUtil.apply(goalPose));

          if (DriverStation.isAutonomousEnabled()) {
            return AllianceFlipUtil.apply(goalPose);
          } else {
            Pose2d flippedGoalPose = AllianceFlipUtil.apply(goalPose);
            Rotation2d originalRotation = flippedGoalPose.getRotation();
            Rotation2d rotationAdjustment =
                AllianceFlipUtil.apply(getBranchPose(objective))
                    .getTranslation()
                    .minus(robot.get().getTranslation())
                    .getAngle()
                    .minus(originalRotation);
            rotationAdjustment =
                Rotation2d.fromDegrees(
                    MathUtil.clamp(
                        rotationAdjustment.getDegrees(),
                        -maxAimingAngle.get(),
                        maxAimingAngle.get()));
            return new Pose2d(
                flippedGoalPose.getTranslation(), originalRotation.plus(rotationAdjustment));
          }
        };

    var driveCommand =
        new DriveToPose(
            driveBase,
            () ->
                coralObjective
                    .get()
                    .map(objective -> getDriveTarget(robot.get(), goal.apply(objective)))
                    .orElseGet(RobotState.getInstance()::getEstimatedPose),
            robot,
            () ->
                DriveCommands.getLinearVelocityFromJoysticks(
                        driverX.getAsDouble(), driverY.getAsDouble())
                    .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
            () -> DriveCommands.getOmegaFromJoysticks(driverOmega.getAsDouble()));

    Timer l4EjectTimer = new Timer();
    l4EjectTimer.start();
    return Commands.runOnce(
            () -> {
              // Start LEDs
              LEDBase.getInstance().autoScoringReef = true;

              // Log reef level
              Logger.recordOutput("AutoScore/ReefLevel", reefLevel.get().toString());

              // Clear logs
              Logger.recordOutput("AutoScore/AllowPreReady", false);
              Logger.recordOutput("AutoScore/AllowEject", false);
            })
        .andThen(
            Commands.waitUntil(
                () -> {
                  boolean ready =
                      coralObjective.get().isPresent()
                              && readyForScore(
                                  robot.get(),
                                  goal.apply(coralObjective.get().get()),
                                  reefLevel.get() == ReefLevel.L4)
                          || disableReefAutoAlign.getAsBoolean();
                  Logger.recordOutput("AutoScore/AllowReady", ready);

                  return ready;
                }),
            aimAndEject(
                elevatorBase,
                dispenserBase,
                reefLevel,
                () -> {
                  if (coralObjective.get().isEmpty()) return false;
                  Pose2d poseError = robot.get().relativeTo(goal.apply(coralObjective.get().get()));

                  int intReefLevel = coralObjective.get().get().reefLevel().ordinal();
                  var driveChassisSpeeds = driveBase.getChassisSpeeds();
                  boolean ready =
                      (Math.abs(poseError.getTranslation().getX())
                                  <= linearXToleranceEject[intReefLevel].get()
                              && Math.abs(poseError.getTranslation().getY())
                                  <= linearYToleranceEject[intReefLevel].get()
                              && Math.abs(poseError.getRotation().getDegrees())
                                  <= thetaToleranceEject[intReefLevel].get()
                              && Math.hypot(
                                      driveChassisSpeeds.vxMetersPerSecond,
                                      driveChassisSpeeds.vyMetersPerSecond)
                                  <= maxLinearVel[intReefLevel].get()
                              && Math.abs(driveChassisSpeeds.omegaRadiansPerSecond)
                                  <= Units.degreesToRadians(maxAngularVel[intReefLevel].get())
                              && elevatorBase.atGoal()
                              && !disableReefAutoAlign.getAsBoolean())
                          || manualEject.getAsBoolean();
                  if (reefLevel.get() == ReefLevel.L4) {
                    if (!ready) {
                      l4EjectTimer.restart();
                    }
                    ready =
                        ready
                            && l4EjectTimer.hasElapsed(
                                DriverStation.isAutonomous()
                                    ? l4EjectDelayAuto.get()
                                    : l4EjectDelay.get());
                  }
                  Logger.recordOutput("AutoScore/AllowEject", ready);
                  return ready;
                },
                manualEject))
        .deadlineFor(
            Commands.either(
                joystickDrive, driveCommand, disableReefAutoAlign)) // Deadline with driving
        .finallyDo(
            interrupted -> {
              // Clear logs
              Logger.recordOutput("AutoScore/ReefLevel", "");
              Logger.recordOutput("AutoScore/AllowPreReady", false);
              Logger.recordOutput("AutoScore/AllowEject", false);

              // Stop LEDs
              LEDBase.getInstance().autoScoringReef = false;
            });
  }

  public static Command autoScore(
      DriveBase driveBase,
      ElevatorBase elevatorBase,
      DispenserBase dispenserBase,
      Supplier<ReefLevel> reefLevel,
      Supplier<Optional<CoralObjective>> coralObjective) {
    return autoScore(
        driveBase,
        elevatorBase,
        dispenserBase,
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

  public static Command aimAndEject(
      ElevatorBase elevatorBase,
      DispenserBase dispenserBase,
      Supplier<ReefLevel> reefLevel,
      BooleanSupplier eject,
      BooleanSupplier holdEject) {
    return elevatorBase
        .runGoal(() -> ElevatorBase.getScoringState(reefLevel.get()))
        .until(eject)
        .andThen(
            elevatorBase
                .runGoal(() -> ElevatorBase.getScoringState(reefLevel.get()))
                .withDeadline(
                    Commands.waitUntil(elevatorBase::atGoal)
                        .andThen(
                            dispenserBase
                                .runDispenser(
                                    () ->
                                        dispenserBase.getDispenserVoltageFromLevel(reefLevel.get()))
                                .withDeadline(
                                    Commands.deferredProxy(
                                            () ->
                                                Commands.waitSeconds(
                                                    ejectTimeSeconds[reefLevel.get().ordinal()]
                                                        .get()))
                                        .andThen(
                                            Commands.waitUntil(() -> !holdEject.getAsBoolean()))))),
            elevatorBase.runGoal(Preset.STOW));
  }

  // private static Command preIntake(
  //     ElevatorBase elevator,
  //     IntakeBase intake,
  //     DispenserBase dispenser,
  //     Supplier<Pose2d> robot,
  //     BooleanSupplier shouldClearReef,
  //     BooleanSupplier disableReefAutoAlign) {
  //   return Commands.waitUntil(
  //           () ->
  //               outOfDistanceToReef(robot.get(), minDistanceReefClearL4.get())
  //                   || !shouldClearReef.getAsBoolean()
  //                   || disableReefAutoAlign.getAsBoolean())
  //       .andThen(IntakeCommands.intake(elevator, intake, dispenser))
  //       .onlyIf(() -> !dispenser.holdingCoral());
  // }

  public static Command autoAlign(
      DriveBase driveBase,
      ElevatorBase elevatorBase,
      Supplier<Optional<CoralObjective>> coralObjective,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega) {
    Supplier<Pose2d> robot =
        () ->
            coralObjective
                .get()
                .map(AutoScoreCommands::getRobotPose)
                .orElseGet(RobotState.getInstance()::getEstimatedPose);

    Function<CoralObjective, Pose2d> goal =
        objective -> {
          if (objective.reefLevel() == ReefLevel.L1) {
            return AllianceFlipUtil.apply(getL1Pose(objective));
          }

          Pose2d goalPose = getCoralScorePose(objective);
          Logger.recordOutput("AutoScore/Goal", AllianceFlipUtil.apply(goalPose));

          if (DriverStation.isAutonomousEnabled()) {
            return AllianceFlipUtil.apply(goalPose);
          } else {
            Pose2d flippedGoalPose = AllianceFlipUtil.apply(goalPose);
            Rotation2d originalRotation = flippedGoalPose.getRotation();
            Rotation2d rotationAdjustment =
                AllianceFlipUtil.apply(getBranchPose(objective))
                    .getTranslation()
                    .minus(robot.get().getTranslation())
                    .getAngle()
                    .minus(originalRotation);
            rotationAdjustment =
                Rotation2d.fromDegrees(
                    MathUtil.clamp(
                        rotationAdjustment.getDegrees(),
                        -maxAimingAngle.get(),
                        maxAimingAngle.get()));
            return new Pose2d(
                flippedGoalPose.getTranslation(), originalRotation.plus(rotationAdjustment));
          }
        };

    return new DriveToPose(
            driveBase,
            () ->
                coralObjective
                    .get()
                    .map(objective -> getDriveTarget(robot.get(), goal.apply(objective)))
                    .orElseGet(RobotState.getInstance()::getEstimatedPose),
            robot,
            () ->
                DriveCommands.getLinearVelocityFromJoysticks(
                        driverX.getAsDouble(), driverY.getAsDouble())
                    .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
            () -> DriveCommands.getOmegaFromJoysticks(driverOmega.getAsDouble()))
        .until(
            () -> {
              if (coralObjective.get().isEmpty()) return false;
              Pose2d poseError = robot.get().relativeTo(goal.apply(coralObjective.get().get()));

              int intReefLevel = coralObjective.get().get().reefLevel().ordinal();
              var driveChassisSpeeds = driveBase.getChassisSpeeds();
              return Math.abs(poseError.getTranslation().getX())
                      <= linearXToleranceEject[intReefLevel].get()
                  && Math.abs(poseError.getTranslation().getY())
                      <= linearYToleranceEject[intReefLevel].get()
                  && Math.abs(poseError.getRotation().getDegrees())
                      <= thetaToleranceEject[intReefLevel].get()
                  && Math.hypot(
                          driveChassisSpeeds.vxMetersPerSecond,
                          driveChassisSpeeds.vyMetersPerSecond)
                      <= maxLinearVel[intReefLevel].get()
                  && Math.abs(driveChassisSpeeds.omegaRadiansPerSecond)
                      <= Units.degreesToRadians(maxAngularVel[intReefLevel].get())
                  && elevatorBase.atGoal();
            });
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
    boolean onRightSide = coralObjective.branchId() % 2 == 0;

    return Reef.centerFaces[coralObjective.branchId() / 2].transformBy(
        new Transform2d(
            DriveConstants.robotWidth / 2.0,
            Reef.faceLength / 2.0 * (onRightSide ? 1 : -1),
            Rotation2d.kPi));
  }

  /** Get position of robot aligned with branch for provided objective. */
  public static Pose2d getCoralScorePose(CoralObjective coralObjective) {
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
}
