package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  // Pitch from Floor to Elevator
  public static final Rotation2d elevatorPitch = Rotation2d.fromDegrees(84.5);
  // Pitch from Elevator to Dispenser
  public static final Rotation2d dispenserPitch = Rotation2d.fromDegrees(73.0);

  public static final double drumRadius = Units.inchesToMeters(1.751 / 2.0);
  public static final double gearing = 3.0;
  public static final int numStages = 2;

  public static final double maxTravel = Units.inchesToMeters(30.0); // TODO

  public static final double carriageMassKg = Units.lbsToKilograms(6.0);
  public static final double stagesMassKg = Units.lbsToKilograms(12.0);
}
