package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.dispenser.DispenserBase;
import frc.robot.subsystems.elevator.ElevatorBase;
import frc.robot.subsystems.elevator.ElevatorPose.Preset;
import frc.robot.subsystems.intake.IntakeBase;
import frc.robot.subsystems.leds.LEDBase;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod({LEDBase.class})
public class IntakeCommands {
  public static Command intake(ElevatorBase elevator, IntakeBase intake, DispenserBase dispenser) {
    final Timer intakeTimer = new Timer();
    return elevator
        .runGoal(Preset.CORAL_INTAKE)
        .alongWith(
            Commands.waitUntil(elevator::atGoal)
                .andThen(
                    dispenser
                        .runDispenser(DispenserBase.coralIntakeVolts)
                        .withDeadline(
                            Commands.waitUntil(dispenser::holdingCoral)
                                .andThen(
                                    Commands.runOnce(intakeTimer::restart),
                                    Commands.waitUntil(
                                            () ->
                                                intakeTimer.hasElapsed(
                                                    DispenserBase.coralIntakeWaitPeriod.get()))
                                        .setLEDState(
                                            (ledBase, state) -> ledBase.coralGrabbed = state))))
                .deadlineFor(intake.runRoller(IntakeBase.intakeVolts)))
        .setLEDState((ledBase, state) -> ledBase.intaking = state)
        .finallyDo(() -> elevator.setGoal(Preset.STOW));
  }
}
