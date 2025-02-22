package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.AutoLog;

public interface EffectorIO {
  @AutoLog
  class EffectorIOInputs {
    public boolean connected = false;

    public double positionRads;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;

    public boolean rearBeamBreakBroken = false;
  }

  default void updateInputs(EffectorIOInputs inputs) {}

  default void runVolts(double output) {}

  default void stop() {}

  default void setBrakeMode(boolean enabled) {}
}
