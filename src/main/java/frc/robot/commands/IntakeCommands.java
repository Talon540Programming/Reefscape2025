package frc.robot.commands;

import frc.robot.util.LoggedTunableNumber;

public class IntakeCommands {
  public static final LoggedTunableNumber intakeVolts =
      new LoggedTunableNumber("Intake/HopperIntakeVolts", 4.0);

  // public static Command intake(SuperstructureBase superstructure, IntakeBase hopper) {
  //   // return
  //   //
  // superstructure.runGoal(SuperstructureState.INTAKE).alongWith(Commands.waitUntil(superstructure::atGoal).andThen(hopper.runRoller(intakeVolts.get())));
  //   return Commands.none();
  // }
}
