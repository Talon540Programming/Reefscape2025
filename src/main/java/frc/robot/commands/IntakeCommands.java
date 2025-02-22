package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeBase;
import frc.robot.subsystems.superstructure.SuperstructureBase;
import frc.robot.util.LoggedTunableNumber;

public class IntakeCommands {
  public static final LoggedTunableNumber intakeVolts =
      new LoggedTunableNumber("Intake/HopperIntakeVolts", 4.0);

  public static Command intake(SuperstructureBase superstructure, IntakeBase hopper) {
    return Commands.none();
  }
}
