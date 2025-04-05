package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
<<<<<<< Updated upstream
  // Pitch from Floor to Elevator
  public static final Rotation2d elevatorPitch = Rotation2d.fromDegrees(86.0);
  // Pitch from Elevator to Dispenser
  public static final Rotation2d dispenserPitch = Rotation2d.fromDegrees(61.6);
=======
  // Measured from CAD
  public static final double stageThickness = Units.inchesToMeters(1.0);
  public static final double slamBlockThickness = Units.inchesToMeters(0.25);
  public static final double dispenserToTop = Units.inchesToMeters(6.214169);
  public static final double dispenserToBottom = Units.inchesToMeters(3.880155);
  public static final double stageToStage = Units.inchesToMeters(1.25);
  // public static final double firstStageHeight = 0.757;
>>>>>>> Stashed changes

  public static final double originToBaseHeightMeters =
      Units.inchesToMeters(6.288); // <-- highest edge on slam plate
  //    6.148 to the lowest edge, 6.218 to centroid

  public static final double drumRadius = Units.inchesToMeters(1.0);
  public static final double gearing = 4.0;

  public static final double maxTravel = Units.inchesToMeters(75); // TODO

  public static final double carriageMassKg = Units.lbsToKilograms(6.0);
  public static final double stagesMassKg = Units.lbsToKilograms(12.0);
}
