package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.util.Container;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.MirrorUtil;
import java.util.Optional;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class AutoBuilder {
  private static final LoggedTunableNumber scoreCancelSecs =
      new LoggedTunableNumber("AutoBuilder/ScoreCancelSeconds", 1.5);
  private static final LoggedTunableNumber intakeTimeSecs =
      new LoggedTunableNumber("AutoBuilder/IntakeTimeSecs", 0.5);

  private static final Pose2d centerStartingPose =
      new Pose2d(
          FieldConstants.startingLineX - DriveConstants.robotWidth / 2.0,
          FieldConstants.fieldWidth / 2.0,
          Rotation2d.kPi);
  private static final Pose2d sideStartingPose =
      new Pose2d(
          FieldConstants.startingLineX - DriveConstants.robotWidth / 2.0,
          FieldConstants.fieldWidth - FieldConstants.Barge.middleCage.getY(),
          Rotation2d.kCCW_Pi_2);

  private final DriveBase driveBase;
  private final ElevatorBase elevatorBase;
  private final DispenserBase dispenserBase;
  private final IntakeBase intakeBase;

  public Command centerStartSingle() {
    final var objective = new CoralObjective(7, ReefLevel.L4);

    return Commands.runOnce(
            () ->
                RobotState.getInstance()
                    .resetPose(AllianceFlipUtil.apply(MirrorUtil.apply(centerStartingPose))))
        .andThen(
            safetyPush(),
            AutoScoreCommands.autoScore(
                driveBase,
                elevatorBase,
                dispenserBase,
                objective::reefLevel,
                () -> Optional.of(MirrorUtil.apply(objective))),
            new DriveToPose(
                    driveBase,
                    () ->
                        AllianceFlipUtil.apply(
                            AutoScoreCommands.getCoralScorePose(objective)
                                .transformBy(new Transform2d(-0.5, 0.0, Rotation2d.kZero))))
                .withTimeout(3.0));
  }

  public Command sideStartMulti(boolean isDeadReckoned) {
    CoralObjective[] coralObjectives =
        new CoralObjective[] {
          new CoralObjective(8, isDeadReckoned ? ReefLevel.L4 : ReefLevel.L2),
          new CoralObjective(10, FieldConstants.ReefLevel.L4),
          new CoralObjective(11, ReefLevel.L4),
          new CoralObjective(0, ReefLevel.L4)
        };
    Container<Integer> currentObjectiveIndex = new Container<>();

    var driveToStation = new DriveToStation(driveBase, true);

    Timer autoTimer = new Timer();
    Timer intakeTimer = new Timer();
    Timer coralIndexedTimer = new Timer();

    return Commands.runOnce(
            () -> {
              RobotState.getInstance()
                  .resetPose(AllianceFlipUtil.apply(MirrorUtil.apply(sideStartingPose)));
              autoTimer.restart();
              currentObjectiveIndex.value = 0;
            })
        .andThen(
            safetyPush(),
            Commands.sequence(
                    // Intake
                    driveToStation
                        .deadlineFor(
                            AutoScoreCommands.aimAndEject(
                                    elevatorBase,
                                    dispenserBase,
                                    () -> coralObjectives[currentObjectiveIndex.value].reefLevel(),
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
                            () -> coralObjectives[currentObjectiveIndex.value].reefLevel(),
                            () ->
                                Optional.of(
                                    MirrorUtil.apply(coralObjectives[currentObjectiveIndex.value])))
                        .withTimeout(1.0)
                        .finallyDo(
                            interrupted -> {
                              if (!interrupted) {
                                System.out.printf(
                                    "Scored Coral #"
                                        + (currentObjectiveIndex.value + 1)
                                        + " at %.2f\n",
                                    autoTimer.get());
                                currentObjectiveIndex.value++;
                              }
                            })
                        .beforeStarting(coralIndexedTimer::restart)
                        .raceWith(
                            Commands.waitUntil(
                                    () -> coralIndexedTimer.hasElapsed(scoreCancelSecs.get()))
                                .andThen(Commands.idle().onlyIf(dispenserBase::holdingCoral))))
                .repeatedly()
                .until(() -> currentObjectiveIndex.value >= coralObjectives.length))
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

  public Command taxi() {
    return Commands.runOnce(
            () ->
                RobotState.getInstance()
                    .resetPose(
                        AllianceFlipUtil.apply(
                            new Pose2d(
                                RobotState.getInstance().getEstimatedPose().getTranslation(),
                                Rotation2d.kPi))))
        .andThen(
            new DriveToPose(
                    driveBase,
                    () -> RobotState.getInstance().getEstimatedPose(),
                    () -> RobotState.getInstance().getEstimatedPose(),
                    () ->
                        new Translation2d((AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0) * -1.0, 0.0),
                    () -> 0.0)
                .withTimeout(0.6));
  }

  private Command safetyPush() {
    return new DriveToPose(
            driveBase,
            () -> {
              var robot = RobotState.getInstance().getEstimatedPose();
              return new Pose2d(
                  robot
                      .getTranslation()
                      .plus(new Translation2d(AllianceFlipUtil.shouldFlip() ? -0.1 : 0.1, 0.0)),
                  robot.getRotation());
            })
        .withTimeout(0.25);
  }
}
