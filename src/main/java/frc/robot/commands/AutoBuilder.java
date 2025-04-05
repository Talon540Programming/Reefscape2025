package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.CoralObjective;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.RobotState;
import frc.robot.subsystems.dispenser.DispenserBase;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.elevator.ElevatorBase;
import frc.robot.subsystems.intake.IntakeBase;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.MirrorUtil;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicInteger;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class AutoBuilder {
  private static final LoggedTunableNumber scoreCancelSecs =
      new LoggedTunableNumber("AutoBuilder/ScoreCancelSeconds", 1.5);
  private static final LoggedTunableNumber intakeTimeSecs =
      new LoggedTunableNumber("AutoBuilder/IntakeTimeSecs", 0.15);

  private static final Pose2d startingPose =
      new Pose2d(
          FieldConstants.startingLineX - DriveConstants.robotWidth / 2.0,
          FieldConstants.fieldWidth
              - FieldConstants.Barge.closeCage.getY()
              + (FieldConstants.Barge.closeCage.getY() - FieldConstants.Barge.middleCage.getY())
                  / 2.0,
          Rotation2d.fromDegrees(150));

  private final DriveBase driveBase;
  private final ElevatorBase elevatorBase;
  private final DispenserBase dispenserBase;
  private final IntakeBase intakeBase;

  public Command taxi() {
    return new DriveToPose(
            driveBase,
            () -> {
              Pose2d robot = RobotState.getInstance().getEstimatedPose();
              return new Pose2d(
                  robot
                      .getTranslation()
                      .plus(new Translation2d(AllianceFlipUtil.shouldFlip() ? 0.1 : -0.1, 0.0)),
                  robot.getRotation());
            })
        .withTimeout(0.5);
  }

  public Command chungusAuto(boolean isDeadReckoned) {
    CoralObjective[] coralObjectives =
        new CoralObjective[] {
          new CoralObjective(8, isDeadReckoned ? ReefLevel.L4 : ReefLevel.L2),
          new CoralObjective(10, FieldConstants.ReefLevel.L4),
          new CoralObjective(11, ReefLevel.L4),
          new CoralObjective(0, ReefLevel.L4)
        };
    return buildCoralAuto(coralObjectives, startingPose);
  }

  public Command painAuto() {
    return Commands.runEnd(
        () -> driveBase.runVelocity(new ChassisSpeeds(3.0, 0, 0)), driveBase::stop);
  }

  public Command superSafeSkibidiL4() {
    CoralObjective[] coralObjectives =
        new CoralObjective[] {new CoralObjective(7, FieldConstants.ReefLevel.L4)};

    return Commands.waitSeconds(2.0)
        .andThen(
            AutoScoreCommands.autoAlign(
                    driveBase,
                    elevatorBase,
                    () -> Optional.of(MirrorUtil.apply(coralObjectives[0])))
                .withTimeout(5.0)
                .andThen(
                    AutoScoreCommands.aimAndEject(
                        elevatorBase,
                        dispenserBase,
                        () -> coralObjectives[0].reefLevel(),
                        () -> true,
                        () -> false)))
        .beforeStarting(
            Commands.runOnce(
                () ->
                    RobotState.getInstance()
                        .resetPose(
                            AllianceFlipUtil.apply(
                                MirrorUtil.apply(
                                    new Pose2d(
                                        FieldConstants.startingLineX
                                            - DriveConstants.robotWidth / 2.0,
                                        FieldConstants.fieldWidth / 2.0,
                                        Rotation2d.kPi))))));
  }

  public Command superSafeSideStartL4() {
    CoralObjective[] coralObjectives =
        new CoralObjective[] {new CoralObjective(8, FieldConstants.ReefLevel.L4)};

    return AutoScoreCommands.autoAlign(
            driveBase, elevatorBase, () -> Optional.of(MirrorUtil.apply(coralObjectives[0])))
        .withTimeout(5.0)
        .andThen(
            AutoScoreCommands.aimAndEject(
                elevatorBase,
                dispenserBase,
                () -> coralObjectives[0].reefLevel(),
                () -> true,
                () -> false))
        .beforeStarting(
            Commands.runOnce(
                () ->
                    RobotState.getInstance()
                        .resetPose(AllianceFlipUtil.apply(MirrorUtil.apply(startingPose)))));
  }

  public Command practiceFieldAuto() {
    CoralObjective[] coralObjectives =
        new CoralObjective[] {
          new CoralObjective(2, ReefLevel.L4),
        };
    return buildCoralAuto(
        coralObjectives,
        FieldConstants.Reef.centerFaces[1].transformBy(new Transform2d(1.5, 0, Rotation2d.kPi)));
  }

  private Command buildCoralAuto(CoralObjective[] coralObjectives, Pose2d startingPose) {
    var currentObjectiveIndex = new AtomicInteger(0);

    var driveToStation = new DriveToStation(driveBase, true);
    Timer autoTimer = new Timer();
    Timer intakeTimer = new Timer();
    Timer coralIndexedTimer = new Timer();

    return Commands.runOnce(
            () -> {
              RobotState.getInstance()
                  .resetPose(AllianceFlipUtil.apply(MirrorUtil.apply(startingPose)));
              autoTimer.restart();
              currentObjectiveIndex.set(0);
            })
        .andThen(
            Commands.sequence(
                    // Intake
                    driveToStation
                        .deadlineFor(
                            AutoScoreCommands.aimAndEject(
                                    elevatorBase,
                                    dispenserBase,
                                    () -> coralObjectives[currentObjectiveIndex.get()].reefLevel(),
                                    () -> true,
                                    () -> true)
                                .until(
                                    () -> {
                                      Pose2d robot = RobotState.getInstance().getEstimatedPose();
                                      Pose2d flippedRobot = AllianceFlipUtil.apply(robot);
                                      return AutoScoreCommands.outOfDistanceToReef(
                                              robot, AutoScoreCommands.minDistanceReefClearL4.get())
                                          || Math.abs(
                                                  FieldConstants.Reef.center
                                                      .minus(flippedRobot.getTranslation())
                                                      .getAngle()
                                                      .minus(flippedRobot.getRotation())
                                                      .getDegrees())
                                              >= AutoScoreCommands.minAngleReefClear.get();
                                    })
                                .andThen(
                                    IntakeCommands.intake(elevatorBase, intakeBase, dispenserBase)))
                        .until(
                            () -> {
                              if (!driveToStation.withinTolerance(
                                  Units.inchesToMeters(5.0), Rotation2d.fromDegrees(5.0))) {
                                intakeTimer.restart();
                              }
                              return dispenserBase.holdingCoral()
                                  || intakeTimer.hasElapsed(intakeTimeSecs.get());
                            }),
                    // Score
                    AutoScoreCommands.autoScore(
                            driveBase,
                            elevatorBase,
                            dispenserBase,
                            intakeBase,
                            () -> coralObjectives[currentObjectiveIndex.get()].reefLevel(),
                            () ->
                                Optional.of(
                                    MirrorUtil.apply(coralObjectives[currentObjectiveIndex.get()])))
                        .finallyDo(
                            interrupted -> {
                              if (!interrupted) {
                                System.out.printf(
                                    "Scored Coral #"
                                        + (currentObjectiveIndex.get() + 1)
                                        + " at %.2f\n",
                                    autoTimer.get());
                                currentObjectiveIndex.set(currentObjectiveIndex.get() + 1);
                              }
                            })
                        .beforeStarting(coralIndexedTimer::restart)
                        .raceWith(
                            Commands.waitUntil(
                                    () -> coralIndexedTimer.hasElapsed(scoreCancelSecs.get()))
                                .andThen(Commands.idle().onlyIf(dispenserBase::holdingCoral))))
                .repeatedly()
                .until(() -> currentObjectiveIndex.get() >= coralObjectives.length))
        .andThen(
            new DriveToPose(
                    driveBase,
                    () ->
                        RobotState.getInstance()
                            .getEstimatedPose()
                            .transformBy(
                                GeomUtil.toTransform2d(
                                    -AutoScoreCommands.minDistanceReefClearL4.get(), 0.0)))
                .until(
                    () ->
                        AutoScoreCommands.outOfDistanceToReef(
                            RobotState.getInstance().getEstimatedPose(),
                            AutoScoreCommands.minDistanceReefClearL4.get())));
  }
}
