// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {

    boolean driveConnected = false;
    double drivePositionRad = 0;
    double driveVelocityRadPerSec = 0;
    double driveAppliedVolts = 0;
    double driveSupplyCurrentAmps = 0;
    double driveTempCelsius = 0;

    boolean turnConnected = false;
    boolean turnEncoderConnected = false;
    Rotation2d turnAbsolutePosition = new Rotation2d();
    Rotation2d turnPosition = new Rotation2d();
    double turnVelocityRadPerSec = 0;
    double turnAppliedVolts = 0;
    double turnSupplyCurrentAmps = 0;
    double turnTempCelsius = 0;

    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified open loop value. */
  public default void runDriveOpenLoop(double output) {}

  /** Run the turn motor at the specified open loop value. */
  public default void runTurnOpenLoop(double output) {}

  /** Run the drive motor at the specified velocity. */
  public default void runDriveVelocity(double velocityRadPerSec, double feedforward) {}

  /** Run the turn motor to the specified rotation. */
  public default void runTurnPosition(Rotation2d rotation) {}

  /** Set P, I, and D gains for closed loop control on drive motor. */
  public default void setDrivePID(double kP, double kI, double kD) {}

  /** Set P, I, and D gains for closed loop control on turn motor. */
  public default void setTurnPID(double kP, double kI, double kD) {}

  /** Set brake mode on drive motor */
  public default void setBrakeMode(boolean enabled) {}
}
