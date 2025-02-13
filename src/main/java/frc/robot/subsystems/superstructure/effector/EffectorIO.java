package frc.robot.subsystems.superstructure.effector;

import org.littletonrobotics.junction.AutoLog;

public interface EffectorIO {
  @AutoLog
  class EffectorIOInputs {
    public boolean connected = false;

    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;

    public boolean rearBeamBreakBroken = false;
  }
}
