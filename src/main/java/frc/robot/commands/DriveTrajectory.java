// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.PoseEstimator;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveTrajectory {
  private static final LoggedTunableNumber drivekP =
      new LoggedTunableNumber("DriveTrajectory/DrivekP");
  private static final LoggedTunableNumber drivekD =
      new LoggedTunableNumber("DriveTrajectory/DrivekD");
  private static final LoggedTunableNumber thetakP =
      new LoggedTunableNumber("DriveTrajectory/ThetakP");
  private static final LoggedTunableNumber thetakD =
      new LoggedTunableNumber("DriveTrajectory/ThetakD");
  private static final LoggedTunableNumber driveMaxVelocity =
      new LoggedTunableNumber("DriveTrajectory/DriveMaxVelocity");
  private static final LoggedTunableNumber driveMaxVelocitySlow =
      new LoggedTunableNumber("DriveTrajectory/DriveMaxVelocitySlow");
  private static final LoggedTunableNumber driveMaxAcceleration =
      new LoggedTunableNumber("DriveTrajectory/DriveMaxAcceleration");
  private static final LoggedTunableNumber thetaMaxVelocity =
      new LoggedTunableNumber("DriveTrajectory/ThetaMaxVelocity");
  private static final LoggedTunableNumber thetaMaxAcceleration =
      new LoggedTunableNumber("DriveTrajectory/ThetaMaxAcceleration");
  private static final LoggedTunableNumber driveTolerance =
      new LoggedTunableNumber("DriveTrajectory/DriveTolerance");
  private static final LoggedTunableNumber thetaTolerance =
      new LoggedTunableNumber("DriveTrajectory/ThetaTolerance");

  static {
    drivekP.initDefault(0.1); // TODO
    drivekD.initDefault(0.0);
    thetakP.initDefault(1.0); // TODO
    thetakD.initDefault(0.0);
    driveMaxVelocity.initDefault(3.8); // TODO
    driveMaxVelocitySlow.initDefault(1); // TODO
    driveMaxAcceleration.initDefault(3.0); // TODO
    thetaMaxVelocity.initDefault(Units.degreesToRadians(360.0));
    thetaMaxAcceleration.initDefault(8.0);
    driveTolerance.initDefault(0.01);
    thetaTolerance.initDefault(Units.degreesToRadians(1.0));
  }

  private final DriveBase drive;

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.kLoopPeriodSecs);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.kLoopPeriodSecs);

//   private final PIDController driveController2 =
//     new PIDController(0.1, 0, 0);

//   private final PIDController thetaController2 =
//     new PIDController(1, 0, 0);

  private Translation2d lastSetpointTranslation = new Translation2d();

  private Supplier<Pose2d> robot = PoseEstimator.getInstance()::getEstimatedPose;

  public DriveTrajectory(DriveBase drive) {
    this.drive = drive;

    // Enable continuous input for theta controller
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void followTrajectory(SwerveSample sample) {
    // Update from tunable numbers
    if (driveMaxVelocity.hasChanged(hashCode())
        || driveMaxVelocitySlow.hasChanged(hashCode())
        || driveMaxAcceleration.hasChanged(hashCode())
        || driveTolerance.hasChanged(hashCode())
        || thetaMaxVelocity.hasChanged(hashCode())
        || thetaMaxAcceleration.hasChanged(hashCode())
        || thetaTolerance.hasChanged(hashCode())
        || drivekP.hasChanged(hashCode())
        || drivekD.hasChanged(hashCode())
        || thetakP.hasChanged(hashCode())
        || thetakD.hasChanged(hashCode())) {
      driveController.setP(drivekP.get());
      driveController.setD(drivekD.get());
      driveController.setConstraints(
          new TrapezoidProfile.Constraints(driveMaxVelocitySlow.get(),
driveMaxAcceleration.get()));
      driveController.setTolerance(driveTolerance.get());
      thetaController.setP(thetakP.get());
      thetaController.setD(thetakD.get());
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()));
      thetaController.setTolerance(thetaTolerance.get());
    }

    // Get current pose and target pose
    Pose2d currentPose = robot.get();

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            sample.vx + driveController.calculate(currentPose.getX(), sample.x),
            sample.vy + driveController.calculate(currentPose.getY(), sample.y),
            sample.omega
                + thetaController.calculate(
                    currentPose.getRotation().getRadians(), sample.heading));

    // Command speeds
    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, currentPose.getRotation()));

    // Log data
    Logger.recordOutput("DriveTrajectory/DistanceSetpoint",
driveController.getSetpoint().position);
    Logger.recordOutput("DriveTrajectory/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("DriveTrajectory/ThetaSetpoint", thetaController.getSetpoint().position);
    Logger.recordOutput(
        "DriveTrajectory/Setpoint",
        new Pose2d[] {
          new Pose2d(
              lastSetpointTranslation,
              Rotation2d.fromRadians(thetaController.getSetpoint().position))
        });
  }
}
