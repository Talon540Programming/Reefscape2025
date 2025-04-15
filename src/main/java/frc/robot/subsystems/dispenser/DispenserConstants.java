package frc.robot.subsystems.dispenser;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class DispenserConstants {
  public static final Rotation2d dispenserAngle = Rotation2d.fromDegrees(-32.5);
  public static final double elevatorToDispenserFront = Units.inchesToMeters(6.0); // TODO

  static final boolean inverted = true;
  static final double moi = 0.025; // TODO
  static final double gearing = 32.0 / 18.0;
}
