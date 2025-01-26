package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.swerve.SwerveSetpointGenerator.ModuleLimits;
import lombok.Builder;

public class DriveConstants {
  public static final double odometryFrequencyHz = 250;

  public static final double trackWidthX = Units.inchesToMeters(20.75);
  public static final double trackWidthY = Units.inchesToMeters(20.75);
  public static final double driveBaseRadius = Math.hypot(trackWidthX / 2, trackWidthY / 2);

  public static final double maxLinearVelocityMetersPerSec = Units.feetToMeters(15.1);
  public static final double maxLinearAccelerationMetersPerSecSquared = Units.feetToMeters(75.0);
  public static final double maxAngularSpeedRadPerSec =
      maxLinearVelocityMetersPerSec / driveBaseRadius;
  public static final double maxAngularAccelerationRadPerSecSquared = 2.0 * Math.PI;

  public static final Translation2d[] moduleTranslations = {
    new Translation2d(trackWidthX / 2, trackWidthY / 2),
    new Translation2d(trackWidthX / 2, -trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, -trackWidthY / 2)
  };

  public static final double wheelRadius = Units.inchesToMeters(2.0);

  private static final double mk4iDriveGearing = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  private static final double mk4iTurnGearing = (150.0 / 7.0);

  public static final ModuleConfig[] moduleConfigs = {
    // FL
    ModuleConfig.builder()
        .turnMotorId(2)
        .driveMotorId(3)
        .encoderChannel(2)
        .encoderOffset(Rotation2d.fromRadians(0.16028737150729522))
        .driveGearing(mk4iDriveGearing)
        .turnGearing(mk4iTurnGearing)
        .turnInverted(true)
        .build(),
    // FR
    ModuleConfig.builder()
        .turnMotorId(4)
        .driveMotorId(5)
        .encoderChannel(3)
        .encoderOffset(Rotation2d.fromRadians(-0.1422097592800296))
        .driveGearing(mk4iDriveGearing)
        .turnGearing(mk4iTurnGearing)
        .turnInverted(true)
        .build(),
    // BL
    ModuleConfig.builder()
        .turnMotorId(6)
        .driveMotorId(7)
        .encoderChannel(1)
        .encoderOffset(Rotation2d.fromRadians(-3.009554996093968))
        .driveGearing(mk4iDriveGearing)
        .turnGearing(mk4iTurnGearing)
        .turnInverted(true)
        .build(),
    // BR
    ModuleConfig.builder()
        .turnMotorId(8)
        .driveMotorId(9)
        .encoderChannel(0)
        .encoderOffset(Rotation2d.fromRadians(2.559973505647124))
        .driveGearing(mk4iDriveGearing)
        .turnGearing(mk4iTurnGearing)
        .turnInverted(true)
        .build(),
  };

  public static class PigeonConstants {
    public static final int id = 10;
  }

  @Builder
  public record ModuleConfig(
      int turnMotorId,
      int driveMotorId,
      int encoderChannel,
      Rotation2d encoderOffset,
      double driveGearing,
      double turnGearing,
      boolean turnInverted) {}

  public static final ModuleLimits moduleLimitsFree =
      new ModuleLimits(
          maxLinearVelocityMetersPerSec,
          maxLinearAccelerationMetersPerSecSquared,
          Units.degreesToRadians(1080.0));
}
