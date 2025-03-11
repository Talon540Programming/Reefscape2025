package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import java.util.Queue;

public class ModuleIOSim implements ModuleIO {
  private final DCMotor driveMotorModel = DCMotor.getNEO(1);
  private final DCMotor turnMotorModel = DCMotor.getNEO(1);

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

  private final PIDController driveController = new PIDController(0, 0, 0);
  private final PIDController turnController = new PIDController(0, 0, 0);
  private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);

  private double driveAppliedVolts = 0.0;
  private double driveFFVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  // Queue inputs from odometry thread
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
    inputs.driveConnected = true;
    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnAbsolutePosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

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
  public void runDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts = driveFeedforward.calculate(velocityRadPerSec);
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void runTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD, double IZone) {
    driveController.setPID(kP, kI, kD);
    driveController.setI(IZone);
  }

  @Override
  public void setDriveFF(double kS, double kV) {
    driveFeedforward.setKs(kS);
    driveFeedforward.setKv(kV);
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    turnController.setPID(kP, kI, kD);
  }
}
