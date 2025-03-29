package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.dispenser.DispenserBase;
import frc.robot.subsystems.elevator.ElevatorBase;
import frc.robot.subsystems.elevator.ElevatorPose.Preset;
import frc.robot.subsystems.intake.IntakeBase;
import frc.robot.subsystems.leds.LEDBase;
import frc.robot.util.LoggedTunableNumber;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod({LEDBase.class})
public class IntakeCommands {
  public static final LoggedTunableNumber intakeVolts =
      new LoggedTunableNumber("Intake/HopperIntakeVolts", 5.5);
  public static final LoggedTunableNumber intakeReserializeVolts =
      new LoggedTunableNumber("Intake/HopperReserializeIntakeVolts", -2.0);

  public static Command intake(ElevatorBase elevator, IntakeBase intake, DispenserBase dispenser) {
    return elevator
        .runGoal(Preset.CORAL_INTAKE)
        .alongWith(
            Commands.waitUntil(elevator::isAtGoal)
                .andThen(dispenser.intakeTillHolding())
                .deadlineFor(intake.runRoller(intakeVolts)))
        .setLEDState((visionBase, state) -> visionBase.intaking = state)
        .finallyDo(() -> elevator.setGoal(Preset.STOW));
  }

  public static Command reserialize(
      ElevatorBase elevator, IntakeBase intake, DispenserBase dispenser) {
    return elevator
        .runGoal(Preset.CORAL_INTAKE)
        .alongWith(
            Commands.waitUntil(elevator::isAtGoal)
                .andThen(
                    dispenser
                        .runDispenser(intakeReserializeVolts)
                        .until(() -> !dispenser.isHoldingCoral())))
        .andThen(intake(elevator, intake, dispenser));
  }
}
