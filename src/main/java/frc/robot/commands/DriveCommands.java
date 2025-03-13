// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.PoseEstimator;
import frc.robot.subsystems.drive.DriveBase;

import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {

  // Drive
  private static final double DEADBAND = 0.1;

  private static final LoggedTunableNumber teleopLinearScalar =
      new LoggedTunableNumber("TeleopDrive/LinearVelocityScalar", 1.0);
  private static final LoggedTunableNumber teleopLinearScalarSlowMode =
      new LoggedTunableNumber("TeleopDrive/LinearVelocityScalarSprint", 0.5);
  private static final LoggedTunableNumber teleopAngularScalar =
      new LoggedTunableNumber("TeleopDrive/AngularVelocityScalar", 1.0);

  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;

  /**
   * Field or robot relative drive command using two joysticks (controlling linear and angular
   * velocities).
   */
  public static Command joystickDrive(
      DriveBase drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier slowSupplier,
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
          double linearVelocityScalar =
              slowSupplier.getAsBoolean()
                  ? teleopLinearScalarSlowMode.get()
                  : teleopLinearScalar.get();
          double angularVelocityScalar = teleopAngularScalar.get();

          var speeds =
              new ChassisSpeeds(
                  x * DriveBase.getMaxLinearVelocityMetersPerSecond() * linearVelocityScalar,
                  y * DriveBase.getMaxLinearVelocityMetersPerSecond() * linearVelocityScalar,
                  omega * DriveBase.getMaxAngularVelocityRadPerSec() * angularVelocityScalar);

          // Convert to field relative
          if (!robotRelativeSupplier.getAsBoolean()) {
            Rotation2d rotation = PoseEstimator.getInstance().getRotation();
            if (AllianceFlipUtil.shouldFlip()) {
              rotation = rotation.rotateBy(Rotation2d.kPi);
            }
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rotation);
          }

          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * DriveBase.getMaxLinearVelocityMetersPerSecond(),
                  linearVelocity.getY() * DriveBase.getMaxLinearVelocityMetersPerSecond(),
                  omega * DriveBase.getMaxAngularVelocityRadPerSec());

          drive.runVelocity(
              robotRelative.getAsBoolean()
                  ? speeds
                  : ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      DriverStation.getAlliance().isPresent()
                              && DriverStation.getAlliance().get() == Alliance.Red
                          ? PoseEstimator.getInstance().getRotation().plus(new Rotation2d(Math.PI))
                          : PoseEstimator.getInstance().getRotation()));
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      DriveBase drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,

      Supplier<Rotation2d> rotationSupplier,
      BooleanSupplier slowSupplier) {
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              Rotation2d rotation = PoseEstimator.getInstance().getRotation();
              double omega =
                  angleController.calculate(
                      rotation.getRadians(), rotationSupplier.get().getRadians());


              // Generate robot relative speeds
              double linearVelocityScalar =
                  slowSupplier.getAsBoolean()
                      ? teleopLinearScalarSlowMode.get()
                      : teleopLinearScalar.get();
              var speeds =
                  new ChassisSpeeds(
                      x * DriveBase.getMaxLinearVelocityMetersPerSecond() * linearVelocityScalar,
                      y * DriveBase.getMaxLinearVelocityMetersPerSecond() * linearVelocityScalar,
                      omega);

              // Convert to field relative
              if (AllianceFlipUtil.shouldFlip()) {
                rotation = rotation.rotateBy(Rotation2d.kPi);
              }
              speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rotation);

              // Apply speeds
              driveBase.runVelocity(speeds);
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(
            () -> angleController.reset(PoseEstimator.getInstance().getRotation().getRadians()));
  }
}
