package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class ElevatorConstants {
  public static final double kElevatorGearing = 3.0 / 1.0;
  public static final double maxElevatorHeightMeters = 1.5617210066;
  public static final double minElevatorHeightMeters = 0.2934208;
  public static final double moi = 0.000367;
  public static final Pose3d elevatorOriginPose3d =
      new Pose3d(new Translation3d(0.089, 0, 0.070), new Rotation3d(84.5, 0, 0));
  public static final Pose3d elevatorExtensionOriginPose3d =
      elevatorOriginPose3d.plus(
          new Transform3d(new Translation3d(0.028123, 0.0, 0.29207), new Rotation3d()));

  public static final int leaderId = 0; // TODO
  public static final int followerId = 0; // TODO
  public static final int leaderEncoderId = 0; // TODO
  public static final int followerEncoderId = 0; // TODO

  public static final int motorCurrentLimit = 40; // TODO
  public static final double encoderPositionFactor = 0.0; // TODO
  public static final double encoderVelocityFactor = 0.0; // TODO

  public static class Sim {
    public static final double kP = 0.0; // TODO
    public static final double kD = 0.0; // TODO
    public static final double kG = 0.0; // TODO
    public static final double kS = 0.0; // TODO
    public static final double kV = 0.0; // TODO
  }

  public static class Real {
    public static final double kP = 0.0; // TODO
    public static final double kD = 0.0; // TODO
    public static final double kG = 0.0; // TODO
    public static final double kS = 0.0; // TODO
    public static final double kV = 0.0; // TODO
  }
}
