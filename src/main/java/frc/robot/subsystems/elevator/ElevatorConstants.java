package frc.robot.subsystems.elevator;

public class ElevatorConstants {
  public static final double kElevatorGearing = 3.0 / 1.0;
  public static final double minHeightMeters = 0;
  public static final double maxHeightMeters = 1.5367 - 0.2681224;
  public static final double startingHeightMeters = 0.2681224;
  public static final double moi = 0.000367;

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
