package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  // Drive
  public static final double DEADBAND = 0.1;

  private static final LoggedTunableNumber teleopLinearScalar =
      new LoggedTunableNumber("TeleopDrive/LinearVelocityScalar", 1.0);
  private static final LoggedTunableNumber teleopAngularScalar =
      new LoggedTunableNumber("TeleopDrive/AngularVelocityScalar", 1.0);

  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      DriveBase driveBase,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier robotRelativeSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), DEADBAND);
          double y = MathUtil.applyDeadband(ySupplier.getAsDouble(), DEADBAND);
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          x = Math.copySign(Math.pow(x, 2), x);
          y = Math.copySign(Math.pow(y, 2), y);
          omega = Math.copySign(Math.pow(omega, 2), omega);

          // Generate robot relative speeds
          double linearVelocityScalar = teleopLinearScalar.get();
          double angularVelocityScalar = teleopAngularScalar.get();

          var speeds =
              new ChassisSpeeds(
                  x * DriveConstants.maxLinearVelocityMetersPerSec * linearVelocityScalar,
                  y * DriveConstants.maxLinearVelocityMetersPerSec * linearVelocityScalar,
                  omega * DriveConstants.maxAngularVelocityRadPerSec * angularVelocityScalar);

          // Convert to field relative
          if (!robotRelativeSupplier.getAsBoolean()) {
            Rotation2d rotation = RobotState.getInstance().getRotation();
            if (AllianceFlipUtil.shouldFlip()) {
              rotation = rotation.rotateBy(Rotation2d.kPi);
            }
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rotation);
          }

          // Apply speeds
          driveBase.runVelocity(speeds);
        },
        driveBase);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      DriveBase driveBase,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              // Apply deadband
              double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), DEADBAND);
              double y = MathUtil.applyDeadband(ySupplier.getAsDouble(), DEADBAND);

              // Square rotation value for more precise control
              x = Math.copySign(Math.pow(x, 2), x);
              y = Math.copySign(Math.pow(y, 2), y);

              // Calculate angular speed
              Rotation2d rotation = RobotState.getInstance().getRotation();
              double omega =
                  angleController.calculate(
                      rotation.getRadians(), rotationSupplier.get().getRadians());

              // Generate robot relative speeds
              double linearVelocityScalar = teleopLinearScalar.get();
              var speeds =
                  new ChassisSpeeds(
                      x * DriveConstants.maxLinearVelocityMetersPerSec * linearVelocityScalar,
                      y * DriveConstants.maxLinearVelocityMetersPerSec * linearVelocityScalar,
                      omega);

              // Convert to field relative
              if (AllianceFlipUtil.shouldFlip()) {
                rotation = rotation.rotateBy(Rotation2d.kPi);
              }
              speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rotation);

              // Apply speeds
              driveBase.runVelocity(speeds);
            },
            driveBase)
        .beforeStarting(
            () -> angleController.reset(RobotState.getInstance().getRotation().getRadians()));
  }

  public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(x, y);

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  public static double getOmegaFromJoysticks(double driverOmega) {
    double omega = MathUtil.applyDeadband(driverOmega, DEADBAND);
    return omega * omega * Math.signum(omega);
  }
}
