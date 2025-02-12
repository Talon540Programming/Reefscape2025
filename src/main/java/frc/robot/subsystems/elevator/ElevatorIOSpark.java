package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.util.SparkUtil;
import java.util.function.DoubleSupplier;

public class ElevatorIOSpark implements ElevatorIO {
  private final SparkBase leader;
  private final SparkBase follower;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;
  private final SparkMaxConfig leaderConfig;
  private final SparkMaxConfig followerConfig;
  private final SparkMaxConfig globalConfig;

  private final Debouncer connectedDebounce = new Debouncer(0.5);

  public ElevatorIOSpark() {
    leader = new SparkMax(ElevatorConstants.leaderId, MotorType.kBrushless);
    follower = new SparkMax(ElevatorConstants.followerId, MotorType.kBrushless);

    encoder = leader.getEncoder();
    controller = leader.getClosedLoopController();

    globalConfig = new SparkMaxConfig();
    globalConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.motorCurrentLimit)
        .voltageCompensation(12);
    globalConfig.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

    leaderConfig = new SparkMaxConfig();
    leaderConfig.apply(globalConfig);
    leaderConfig
        .encoder
        .positionConversionFactor(ElevatorConstants.encoderPositionFactor)
        .velocityConversionFactor(ElevatorConstants.encoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    leaderConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(ElevatorConstants.Real.kP, 0, ElevatorConstants.Real.kD, 0);
    leaderConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20);

    SparkUtil.tryUntilOk(
        leader,
        5,
        () ->
            leader.configure(
                leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    followerConfig = new SparkMaxConfig();
    followerConfig.apply(globalConfig).follow(leader);

    SparkUtil.tryUntilOk(
        follower,
        5,
        () ->
            follower.configure(
                followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    SparkUtil.ifOk(leader, encoder::getPosition, (position) -> inputs.positionRad = position);

    SparkUtil.ifOk(leader, encoder::getVelocity, (vel) -> inputs.velocityRadPerSec = vel);

    SparkUtil.ifOk(
        leader,
        new DoubleSupplier[] {leader::getAppliedOutput, leader::getBusVoltage},
        (values) -> inputs.appliedVolts[0] = values[0] * values[1]);

    SparkUtil.ifOk(
        follower,
        new DoubleSupplier[] {follower::getAppliedOutput, follower::getBusVoltage},
        (values) -> inputs.appliedVolts[1] = values[0] * values[1]);

    SparkUtil.ifOk(leader, leader::getOutputCurrent, (current) -> inputs.currentAmps[0] = current);
    SparkUtil.ifOk(
        follower, follower::getOutputCurrent, (current) -> inputs.currentAmps[1] = current);

    inputs.connected = connectedDebounce.calculate(true);
  }

  @Override
  public void runOpenLoop(double output) {
    leader.setVoltage(output);
  }

  @Override
  public void runPosition(double positionRad, double feedforward) {
    controller.setReference(
        positionRad,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot1,
        feedforward,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }
} // TODO
