// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import java.util.Queue;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class ModuleIOSim implements ModuleIO {
  private static final DCMotor driveMotorModel = DCMotor.getNEO(1);
  private static final DCMotor turnMotorModel = DCMotor.getNEO(1);

  private final DCMotorSim driveSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              driveMotorModel, 0.025, DriveConstants.mk4iDriveGearing),
          driveMotorModel);
  private final DCMotorSim turnSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(turnMotorModel, 0.004, DriveConstants.mk4iTurnGearing),
          turnMotorModel);

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private PIDController driveController = new PIDController(0, 0, 0);
  private PIDController turnController = new PIDController(0, 0, 0);
  private double driveFFVolts = 0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  public ModuleIOSim() {
    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);

    drivePositionQueue =
        OdometryManager.getInstance().registerSignal(driveSim::getAngularPositionRad);
    turnPositionQueue =
        OdometryManager.getInstance().registerSignal(turnSim::getAngularPositionRad);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
    } else {
      turnController.reset();
    }

    // Update simulation state
    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
    driveSim.update(Constants.kLoopPeriodSecs);
    turnSim.update(Constants.kLoopPeriodSecs);

    // Update drive inputs
    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveSupplyCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());
    inputs.driveTempCelsius = 0;

    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnSupplyCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
    inputs.turnTempCelsius = 0;

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    drivePositionQueue.clear();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream().map(Rotation2d::fromRadians).toArray(Rotation2d[]::new);
    turnPositionQueue.clear();
  }

  @Override
  public void runDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void runTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void runDriveVelocity(double velocityRadPerSec, double feedforward) {
    driveClosedLoop = true;
    driveFFVolts = feedforward;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void runTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveController.setPID(kP, kI, kD);
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    turnController.setPID(kP, kI, kD);
  }
}
