package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.swerve.SwerveSetpointGenerator.ModuleLimits;
import lombok.Builder;

public class DriveConstants {
  static final double odometryFrequencyHz = Constants.getMode() == Constants.Mode.SIM ? 50 : 250;

  static final double trackWidthX = Units.inchesToMeters(20.75);
  static final double trackWidthY = Units.inchesToMeters(20.75);
  static final double driveBaseRadius = Math.hypot(trackWidthX / 2, trackWidthY / 2);

  public static final double maxLinearVelocityMetersPerSec = Units.feetToMeters(15.1);
  public static final double maxLinearAccelerationMetersPerSecSquared = Units.feetToMeters(75.0);
  public static final double maxAngularVelocityRadPerSec =
      maxLinearVelocityMetersPerSec / driveBaseRadius;
  public static final double maxAngularAccelerationRadPerSecSquared = 2.0 * Math.PI;

  public static final double driveWidth = Units.inchesToMeters(26.5);
  public static final double bumperThickness = Units.inchesToMeters(3.0);
  public static final double robotWidth = driveWidth + (2 * bumperThickness);

  public static final Translation2d[] moduleTranslations = {
    new Translation2d(trackWidthX / 2, trackWidthY / 2),
    new Translation2d(trackWidthX / 2, -trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, -trackWidthY / 2)
  };

  static final double wheelRadius = Units.inchesToMeters(2.0);

  static final double mk4iDriveGearing = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  static final double mk4iTurnGearing = (150.0 / 7.0);

  static final ModuleConfig[] moduleConfigs = {
    // FL
    ModuleConfig.builder()
        .turnMotorId(2)
        .driveMotorId(3)
        .encoderChannel(0)
        .encoderOffset(Rotation2d.fromRadians(1.7182357115138978).rotateBy(Rotation2d.kPi))
        .driveGearing(mk4iDriveGearing)
        .turnGearing(mk4iTurnGearing)
        .turnInverted(true)
        .build(),
    // FR
    ModuleConfig.builder()
        .turnMotorId(4)
        .driveMotorId(5)
        .encoderChannel(1)
        .encoderOffset(Rotation2d.fromRadians(-1.4361935561244243).rotateBy(Rotation2d.kPi))
        .driveGearing(mk4iDriveGearing)
        .turnGearing(mk4iTurnGearing)
        .turnInverted(true)
        .build(),
    // BL
    ModuleConfig.builder()
        .turnMotorId(6)
        .driveMotorId(7)
        .encoderChannel(2)
        .encoderOffset(Rotation2d.fromRadians(0.9998617084472845).rotateBy(Rotation2d.kPi))
        .driveGearing(mk4iDriveGearing)
        .turnGearing(mk4iTurnGearing)
        .turnInverted(true)
        .build(),
    // BR
    ModuleConfig.builder()
        .turnMotorId(8)
        .driveMotorId(9)
        .encoderChannel(3)
        .encoderOffset(Rotation2d.fromRadians(-1.7285578862473199).rotateBy(Rotation2d.kPi))
        .driveGearing(mk4iDriveGearing)
        .turnGearing(mk4iTurnGearing)
        .turnInverted(true)
        .build(),
  };

  static class PigeonConstants {
    public static final int id = 10;
  }

  @Builder
  record ModuleConfig(
      int turnMotorId,
      int driveMotorId,
      int encoderChannel,
      Rotation2d encoderOffset,
      double driveGearing,
      double turnGearing,
      boolean turnInverted) {}

  static final ModuleLimits moduleLimitsFree =
      new ModuleLimits(
          maxLinearVelocityMetersPerSec,
          maxLinearAccelerationMetersPerSecSquared,
          Units.degreesToRadians(1080.0));
}
