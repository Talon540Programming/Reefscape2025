package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.util.Debouncer;

public class ElevatorIOSpark implements ElevatorIO {
  private final SparkBase leaderSpark;
  private final SparkBase followerSpark;

  private final RelativeEncoder encoder;

  private final SparkClosedLoopController controller;

  private final Debouncer leaderConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer followerConnectedDebouncer = new Debouncer(0.5);

  public ElevatorIOSpark() {

    leaderSpark = new SparkMax(leaderId, MotorType.kBrushless);
    followerSpark = new SparkMax(followerId, MotorType.kBrushless);

    encoder = leaderSpark.getEncoder();
    controller = leaderSpark.getClosedLoopController();

    var leaderConfig = new SparkMaxConfig();
    leaderConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).voltageCompensation(12.0);
    leaderConfig
        .encoder
        .positionConversionFactor(2 * Math.PI / gearing)
        .velocityConversionFactor(2 * Math.PI / 60.0 / gearing)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    leaderConfig
        .closedLoop
        .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        .pidf(0.0, 0.0, 0.0, 0.0);
    leaderConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20)
        .motorTemperaturePeriodMs(20);

    leaderSpark.configure(
        leaderConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    var followerConfig = new SparkMaxConfig();
    followerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(60)
        .voltageCompensation(12.0)
        .follow(leaderSpark, true);

    followerConfig
        .signals
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20)
        .motorTemperaturePeriodMs(20);

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
          leaderSpark.getAppliedOutput() * leaderSpark.getBusVoltage(),
          followerSpark.getAppliedOutput() * followerSpark.getBusVoltage()
        };
    inputs.currentAmps =
        new double[] {leaderSpark.getOutputCurrent(), followerSpark.getOutputCurrent()};
    inputs.tempCelsius =
        new double[] {leaderSpark.getMotorTemperature(), followerSpark.getMotorTemperature()};

    inputs.leaderConnected = leaderConnectedDebouncer.calculate(!leaderSpark.hasActiveFault());
    inputs.followerConnected =
        followerConnectedDebouncer.calculate(!followerSpark.hasActiveFault());
  }

  @Override
  public void runOpenLoop(double output) {
    leaderSpark.setVoltage(output);
  }

  @Override
  public void runPosition(double positionRads, double feedforwardVolts) {
    controller.setReference(
        positionRads,
        SparkBase.ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        feedforwardVolts,
        SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    leaderSpark.stopMotor();
  }

  @Override
  public void resetOrigin() {
    encoder.setPosition(0.0);
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
  public void setBrakeMode(boolean enabled) {
    var brakeModeConfig = new SparkMaxConfig();
    brakeModeConfig.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);

    leaderSpark.configure(
        brakeModeConfig,
        SparkBase.ResetMode.kNoResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);
    followerSpark.configure(
        brakeModeConfig,
        SparkBase.ResetMode.kNoResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);
  }
}
