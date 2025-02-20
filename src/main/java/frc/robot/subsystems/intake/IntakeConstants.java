package frc.robot.subsystems.intake;

class IntakeConstants {
  public static final boolean intakeInverted = true;
  public static final double intakeMoI = 0.025;

  public static final double intakeGearing = 2.0;

  public static final double intakePositionConversionFactor = 2 * Math.PI / intakeGearing;
  public static final double intakeVelocityConversionFactor = intakePositionConversionFactor / 60.0;
}
