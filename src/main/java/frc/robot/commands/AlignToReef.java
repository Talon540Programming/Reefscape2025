// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class AlignToReef extends Command {
  private static final LoggedTunableNumber drivekP = new LoggedTunableNumber("DriveToPose/DrivekP");
  private static final LoggedTunableNumber drivekD = new LoggedTunableNumber("DriveToPose/DrivekD");
  private static final LoggedTunableNumber driveMaxVelocity =
      new LoggedTunableNumber("DriveToPose/DriveMaxVelocity");
  private static final LoggedTunableNumber driveMaxVelocitySlow =
      new LoggedTunableNumber("DriveToPose/DriveMaxVelocitySlow");
  private static final LoggedTunableNumber driveMaxAcceleration =
      new LoggedTunableNumber("DriveToPose/DriveMaxAcceleration");
  private static final LoggedTunableNumber driveTolerance =
      new LoggedTunableNumber("DriveToPose/DriveTolerance");
  private static final LoggedTunableNumber ffMinRadius =
      new LoggedTunableNumber("DriveToPose/FFMinRadius");
  private static final LoggedTunableNumber ffMaxRadius =
      new LoggedTunableNumber("DriveToPose/FFMaxRadius");

  static {
    drivekP.initDefault(0.01);
    drivekD.initDefault(0.0);
    driveMaxVelocity.initDefault(1);
    driveMaxVelocitySlow.initDefault(2);
    driveMaxAcceleration.initDefault(3.0);
    driveTolerance.initDefault(10);
    ffMinRadius.initDefault(0.05);
    ffMaxRadius.initDefault(0.1);
  }

  private final DriveBase drive;
  private final Supplier<Double> target;

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.kLoopPeriodSecs);

  private double lastSetpointTranslation = 0.0;
  private double driveErrorAbs = 0.0;
  @Getter private boolean running = false;
  private Supplier<Double> robot;

  private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;

  public AlignToReef(DriveBase drive, Supplier<Double> target) {
    this.drive = drive;
    this.target = target;

    addRequirements(drive);
  }

  public AlignToReef(DriveBase drive, Supplier<Double> target, Supplier<Double> robot) {
    this(drive, target);
    this.robot = robot;
  }

  public AlignToReef(
      DriveBase drive,
      Supplier<Double> target,
      Supplier<Double> robot,
      Supplier<Translation2d> linearFF,
      DoubleSupplier omegaFF) {
    this(drive, target, robot);
    this.linearFF = linearFF;
  }

  @Override
  public void initialize() {
    double currentPose = robot.get();
    ChassisSpeeds fieldVelocity = drive.getChassisSpeeds();
    double linearFieldVelocity = fieldVelocity.vyMetersPerSecond;
    driveController.reset(currentPose - target.get(), Math.min(0.0, -linearFieldVelocity));
    lastSetpointTranslation = currentPose;
  }

  @Override
  public void execute() {
    running = true;
    Logger.recordOutput("Vision/currentPose", robot.get());
    Logger.recordOutput("Vision/targetPose", target.get());

    // Update from tunable numbers
    if (driveMaxVelocity.hasChanged(hashCode())
        || driveMaxVelocitySlow.hasChanged(hashCode())
        || driveMaxAcceleration.hasChanged(hashCode())
        || driveTolerance.hasChanged(hashCode())
        || drivekP.hasChanged(hashCode())
        || drivekD.hasChanged(hashCode())) {
      driveController.setP(drivekP.get());
      driveController.setD(drivekD.get());
      driveController.setConstraints(
          new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()));
      driveController.setTolerance(driveTolerance.get());
    }

    // Get current pose and target pose
    double currentPose = robot.get();
    double targetPose = target.get();

    // Calculate drive speed
    double currentDistance = currentPose - targetPose;
    // System.out.println(currentDistance);
    double ffScaler =
        MathUtil.clamp(
            (Math.abs(currentDistance) - ffMinRadius.get())
                / (ffMaxRadius.get() - ffMinRadius.get()),
            0.0,
            1.0);
    driveErrorAbs = currentDistance;
    driveController.reset(
        lastSetpointTranslation - targetPose, driveController.getSetpoint().velocity);
    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(driveErrorAbs, 0.0);
    System.out.println(driveVelocityScalar);
    if (Math.abs(currentDistance) < driveController.getPositionTolerance())
      driveVelocityScalar = 0.0;
    lastSetpointTranslation = targetPose + driveController.getSetpoint().position;

    double driveVelocity = driveVelocityScalar;

    // Scale feedback velocities by input ff
    final double linearS = linearFF.get().getNorm() * 3.0;
    driveVelocity =
        MathUtil.interpolate(
            driveVelocity, linearFF.get().times(driveMaxVelocity.get()).getX(), linearS);

    // Command speeds
    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(0, driveVelocity, 0, new Rotation2d()));

    // Log data
    Logger.recordOutput("DriveToPose/DistanceMeasured", currentDistance);
    Logger.recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
    Logger.recordOutput("DriveToPose/Setpoint", new double[] {lastSetpointTranslation});
    Logger.recordOutput("DriveToPose/Goal", new double[] {targetPose});
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    running = false;
    // Clear logs
    Logger.recordOutput("DriveToPose/Setpoint", new Pose2d[] {});
    Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {});
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return running && driveController.atGoal();
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance(double driveTolerance) {
    return running && Math.abs(driveErrorAbs) < driveTolerance;
  }
}
