package frc.robot.subsystems.effector;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  public static class RollerIOInputs {
    public boolean connected = false;
    public boolean hasCoral = false;

    public double appliedVolts = 0.0;
    public double velocityRadPerSec = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(RollerIOInputs inputs) {}

  public default void runOpenLoop(double output) {}

  public default void stop() {}
}
