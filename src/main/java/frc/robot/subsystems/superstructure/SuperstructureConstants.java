package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

class SuperstructureConstants {
  public static final double elevatorGearing = 3.0;
  public static final double elevatorDrumRadius = Units.inchesToMeters(0.8755);

  // Pitch from Floor to Elevator
  public static final Rotation2d elevatorPitch = Rotation2d.fromDegrees(84.5);
  // Pitch from Elevator to End Effector
  public static final Rotation2d effectorPitch = Rotation2d.fromDegrees(73.0);

  public static final double maxElevatorHeightMeters = 1.5617210066; // TODO
  public static final double minElevatorHeightMeters = 0.2934208; // TODO
  public static final double L1_STATE = 0.5; // TODO
  public static final double L2_STATE = 1; // TODO
  public static final double L3_STATE = 1.5; // TODO
  public static final Pose3d elevatorOriginPose3d =
      new Pose3d(new Translation3d(0.089, 0, 0.070), new Rotation3d(84.5, 0, 0)); // TODO
  public static final Pose3d elevatorExtensionOriginPose3d =
      elevatorOriginPose3d.plus(
          new Transform3d(new Translation3d(0.028123, 0.0, 0.29207), new Rotation3d())); // TODO

  public static final boolean effectorInverted = true;
  public static final double effectorMoI = 0.025; // TODO

  public static final double effectorGearing = 34.0 / 24.0;

  public static final double effectorPositionConversionFactor = 2 * Math.PI / effectorGearing;
  public static final double effectorVelocityConversionFactor =
      effectorPositionConversionFactor / 60.0;

  public static final double encoderPositionConversionFactor =
      2 * Math.PI / elevatorGearing; // TODO
  public static final double encoderVelocityConversionFactor =
      encoderPositionConversionFactor / 60.0; // TODO
}
