package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean connected = false;
    public boolean collapsedBeamBreakBroken = false;
    public boolean extendedBeamBreakBroken = false;

    public double appliedVolts = 0.0;
    public double velocityRadPerSec = 0.0;
    public double positionMeters = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void stop() {}

  public default void setBrakeMode(boolean enable) {}
}
