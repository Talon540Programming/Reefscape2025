package frc.robot.subsystems.intake;

class IntakeConstants {
  public static final boolean inverted = true;
  public static final double moi = 0.025;

  public static final double gearing = 2.0;

  public static final double positionConversionFactor = 2 * Math.PI / gearing;
  public static final double velocityConversionFactor = positionConversionFactor / 60.0;
}
