package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.dispenser.DispenserBase;
import frc.robot.subsystems.elevator.ElevatorBase;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.intake.IntakeBase;
import frc.robot.subsystems.leds.LEDBase;
import frc.robot.util.LoggedTunableNumber;

public class IntakeCommands {
  public static final LoggedTunableNumber intakeVolts =
      new LoggedTunableNumber("Intake/HopperIntakeVolts", 5.5);

  public static Command intake(ElevatorBase elevator, IntakeBase intake, DispenserBase dispenser) {
    return Commands.runOnce(() -> elevator.setGoal(ElevatorState.INTAKE))
        .andThen(
            Commands.waitUntil(elevator::isAtGoal)
                .andThen(
                    Commands.deadline(
                        dispenser.intakeTillHolding(), intake.runRoller(intakeVolts.get()))))
        .beforeStarting(() -> LEDBase.getInstance().humanPlayerAlert = true)
        .finallyDo(
            () -> {
              elevator.setGoal(ElevatorState.STOW);
              LEDBase.getInstance().humanPlayerAlert = false;
            });
  }

  public static Command reserialize(
      ElevatorBase elevator, IntakeBase intake, DispenserBase dispenser) {
    return Commands.runOnce(() -> elevator.setGoal(ElevatorState.INTAKE))
        .andThen(
            Commands.waitUntil(elevator::isAtGoal)
                .andThen(dispenser.runRollers(-2.0).until(() -> !dispenser.isHoldingCoral()))
                .andThen(IntakeCommands.intake(elevator, intake, dispenser)));
  }
}
