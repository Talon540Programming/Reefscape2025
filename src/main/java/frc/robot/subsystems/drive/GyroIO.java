package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;

    public Rotation2d yawPosition;
    public double yawVelocityRadPerSec;
    public Rotation2d pitchPosition;
    public double pitchVelocityRadPerSec;
    public Rotation2d rollPosition;
    public double rollVelocityRadPerSec;

    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}
