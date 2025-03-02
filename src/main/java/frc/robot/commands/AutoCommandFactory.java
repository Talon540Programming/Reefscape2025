package frc.robot.commands;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.PoseEstimator;
import frc.robot.subsystems.dispenser.DispenserBase;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.elevator.ElevatorBase;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.intake.IntakeBase;
import frc.robot.util.AllianceFlipUtil;

public class AutoCommandFactory {
  private final DriveBase drive;
  private final IntakeBase intake;
  private final DispenserBase dispenser;
  private final ElevatorBase elevator;

  private final AutoFactory autoFactory;

  public AutoCommandFactory(
      DriveBase drive, IntakeBase intake, DispenserBase dispenser, ElevatorBase elevator) {
    this.drive = drive;
    this.intake = intake;
    this.dispenser = dispenser;
    this.elevator = elevator;

    this.autoFactory =
        new AutoFactory(
            PoseEstimator.getInstance()::getOdometryPose,
            PoseEstimator.getInstance()::resetPose,
            drive::followTrajectory,
            !AllianceFlipUtil.shouldFlip(),
            drive);
  }

  public Command centerToBackL2() {
    return Commands.sequence(
        autoFactory.resetOdometry("centerToBackL2"),
        Commands.deadline(
            autoFactory.trajectoryCmd("centerToBackL2"),
            Commands.runOnce(() -> elevator.setGoal(ElevatorState.L2_CORAL))
            ),
            dispenser.eject()
            );
  }

  public Command rightToFrontL2() {
    return Commands.sequence(
        autoFactory.resetOdometry("rightToFrontL2"),
        Commands.deadline(
            autoFactory.trajectoryCmd("rightToFrontL2"),
            Commands.runOnce(() -> elevator.setGoal(ElevatorState.L2_CORAL))
                .andThen(dispenser.eject())));
  }

  public Command rightL2FeederL2() {
    return Commands.sequence(
        autoFactory.resetOdometry("rightL2FeederL2"),
autoFactory.trajectoryCmd("rightL2FeederL2"));
  }
}
