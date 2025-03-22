package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.dispenser.DispenserBase;
import frc.robot.subsystems.elevator.ElevatorBase;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.intake.IntakeBase;
import frc.robot.util.LoggedTunableNumber;

public class IntakeCommands {
  public static final LoggedTunableNumber intakeVolts =
      new LoggedTunableNumber("Intake/HopperIntakeVolts", 5.5);
  public static final LoggedTunableNumber reserializeVolts =
      new LoggedTunableNumber("Intake/ReserializeVolts", -3);

  public static Command intake(ElevatorBase elevator, IntakeBase intake, DispenserBase dispenser) {
    return Commands.runOnce(() -> elevator.setGoal(ElevatorState.INTAKE))
        .andThen(
            Commands.waitUntil(elevator::isAtGoal)
                .andThen(
                    Commands.deadline(
                        dispenser.intakeTillHolding(), intake.runRoller(intakeVolts.get()))))
        .finallyDo(() -> elevator.setGoal(ElevatorState.STOW));
  }

  public static Command unintake(
      ElevatorBase elevator, IntakeBase intake, DispenserBase dispenser) {
    return Commands.runOnce(() -> elevator.setGoal(ElevatorState.INTAKE))
        .andThen(
            Commands.waitUntil(elevator::isAtGoal)
                .andThen(
                    Commands.deadline(
                        dispenser.intakeTillHolding(), intake.runRoller(reserializeVolts.get()))))
        .finallyDo(() -> elevator.setGoal(ElevatorState.STOW));
  }

  public static Command reserialize(
      ElevatorBase elevator, IntakeBase intake, DispenserBase dispenser) {
    return Commands.runOnce(() -> elevator.setGoal(ElevatorState.INTAKE))
        .andThen(
            Commands.waitUntil(elevator::isAtGoal)
                .andThen(IntakeCommands.unintake(elevator, intake, dispenser))
                .andThen(IntakeCommands.intake(elevator, intake, dispenser)));
  }
}
