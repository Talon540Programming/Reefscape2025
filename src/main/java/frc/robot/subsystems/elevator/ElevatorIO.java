package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean connected = true;
    public boolean collapsedBeamBreakBroken = false;
    public boolean extendedBeamBreakBroken = false;

    public double[] appliedVolts = new double[] {};
    public double velocityRadPerSec = 0.0;
    public double positionRad = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void runPosition(double positionRad, double feedforward) {}

  public default void runOpenLoop(double output) {}

  public default void stop() {}
}
