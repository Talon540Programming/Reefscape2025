package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  // Pitch from Floor to Elevator
  static final Rotation2d elevatorPitch = Rotation2d.fromDegrees(86.0);
  // Pitch from Elevator to Dispenser
  static final Rotation2d dispenserPitch = Rotation2d.fromDegrees(61.6);

  static final double originToBaseHeightMeters = Units.inchesToMeters(6.288);

  static final double drumRadius = Units.inchesToMeters(1.0);
  static final double gearing = 4.0;

  static final double maxTravel = 1.631;

  static final double carriageMassKg = Units.lbsToKilograms(6.0);
  static final double stagesMassKg = Units.lbsToKilograms(12.0);
}
