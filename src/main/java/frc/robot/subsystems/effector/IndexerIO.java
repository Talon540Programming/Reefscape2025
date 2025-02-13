package frc.robot.subsystems.effector;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public boolean connected = false;
    public double appliedVolts = 0.0;
    public double velocityRadPerSec = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void runOpenLoop(double output) {}

  public default void stop() {}
}
