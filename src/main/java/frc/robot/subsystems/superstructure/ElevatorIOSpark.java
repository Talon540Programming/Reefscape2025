package frc.robot.subsystems.superstructure;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;

public class ElevatorIOSpark implements ElevatorIO {
  private final SparkBase leaderSpark;
  private final SparkBase followerSpark;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public ElevatorIOSpark() {
    leaderSpark = new SparkMax(13, SparkLowLevel.MotorType.kBrushless); // TODO: deviceId
    followerSpark = new SparkMax(14, SparkLowLevel.MotorType.kBrushless); // TODO: deviceId

    encoder = leaderSpark.getEncoder();
    controller = leaderSpark.getClosedLoopController();
    var globalConfig = new SparkMaxConfig();
    globalConfig
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .smartCurrentLimit(40)
        .voltageCompensation(12);
    globalConfig.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

    var leaderConfig = new SparkMaxConfig();
    leaderConfig.apply(globalConfig);
    leaderConfig
        .encoder
        .positionConversionFactor(encoderPositionConversionFactor)
        .velocityConversionFactor(encoderVelocityConversionFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    leaderConfig
        .closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .pidf(0, 0, 0, 0);
    leaderConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20);

    var followerConfig = new SparkMaxConfig();
    followerConfig.apply(globalConfig).follow(leaderSpark);

    leaderSpark.configure(
        leaderConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    followerSpark.configure(
        followerConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.positionRads = encoder.getPosition();
    inputs.velocityRadPerSec = encoder.getVelocity();
    inputs.appliedVolts =
        new double[] {
          leaderSpark.getAppliedOutput() * leaderSpark.getAppliedOutput(),
          followerSpark.getAppliedOutput() * followerSpark.getBusVoltage()
        };
    inputs.currentAmps =
        new double[] {leaderSpark.getOutputCurrent(), followerSpark.getOutputCurrent()};
    inputs.tempCelsius =
        new double[] {leaderSpark.getMotorTemperature(), followerSpark.getMotorTemperature()};

    inputs.leaderConnected = connectedDebouncer.calculate(!leaderSpark.hasActiveFault());
    inputs.followerConnected = connectedDebouncer.calculate(!followerSpark.hasActiveFault());
  }

  @Override
  public void runOpenLoop(double output) {
    leaderSpark.setVoltage(output);
  }

  @Override
  public void runPosition(double positionRads, double feedforward) {
    controller.setReference(
        positionRads,
        SparkBase.ControlType.kPosition,
        ClosedLoopSlot.kSlot1,
        feedforward,
        SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    var PIDConfig = new SparkMaxConfig();
    PIDConfig.closedLoop.pid(kP, kI, kD);

    leaderSpark.configure(
        PIDConfig,
        SparkBase.ResetMode.kNoResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);
  }

  @Override
  public void resetOrigin() {
    encoder.setPosition(0.0);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    var brakeModeConfig = new SparkMaxConfig();
    brakeModeConfig.idleMode(
        enabled ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);

    leaderSpark.configure(
        brakeModeConfig,
        SparkBase.ResetMode.kNoResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);
  }
}
