package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public class ElevatorIOInputs {
    public boolean leaderConnected = false;
    public boolean followerConnected = false;

    public double positionRads = 0.0;
    public double velocityRadPerSec = 0.0;
    public double[] appliedVolts = new double[] {};
    public double[] currentAmps = new double[] {};
    public double[] tempCelsius = new double[] {};
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void runOpenLoop(double output) {}

  default void runPosition(double positionRads, double feedforwardVolts) {}

  default void stop() {}

  default void resetOrigin() {}

  default void setPID(double kP, double kI, double kD) {}

  default void setBrakeMode(boolean enabled) {}
}
