package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  // Measured from CAD
  public static final double stageThickness = Units.inchesToMeters(1.0);
  public static final double slamBlockThickness = Units.inchesToMeters(0.25);
  public static final double dispenserToTop = Units.inchesToMeters(6.214169);
  public static final double dispenserToBottom = Units.inchesToMeters(3.880155);

  // Positions of both superstructure and dispenser origin on robot (x forward from center, y off
  // the ground)
  public static final Rotation2d elevatorAngle = Rotation2d.fromDegrees(86.0);
  public static final Translation2d elevatorOrigin2d = new Translation2d(0.08939702, 0.12879159);
  public static final Translation3d elevatorOrigin3d =
      new Translation3d(elevatorOrigin2d.getX(), 0.0, elevatorOrigin2d.getY());

  public static final Translation2d dispenserOrigin2d =
      elevatorOrigin2d.plus(
          new Translation2d(
              dispenserToBottom + (stageThickness * 2) + slamBlockThickness, elevatorAngle));
  public static final Translation3d dispenserOrigin3d =
      new Translation3d(dispenserOrigin2d.getX(), 0.0, dispenserOrigin2d.getY());

  public static final double elevatorMaxTravel = 1.631;

  static final double drumRadius = Units.inchesToMeters(1.0);
  static final double gearing = 4.0;

  static final double carriageMassKg = Units.lbsToKilograms(8.0); // TODO
  static final double stagesMassKg = Units.lbsToKilograms(12.0); // TODO
}
