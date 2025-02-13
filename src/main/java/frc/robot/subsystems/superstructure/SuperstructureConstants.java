package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

class SuperstructureConstants {
  public static final double elevatorGearing = 3.0;
  public static final double elevatorDrumRadius = Units.inchesToMeters(0.8755);

  // Pitch from Floor to Elevator
  public static final Rotation2d elevatorPitch = Rotation2d.fromDegrees(84.5);
  // Pitch from Elevator to End Effector
  public static final Rotation2d effectorPitch = Rotation2d.fromDegrees(73.0);
}
