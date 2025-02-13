package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

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
import frc.robot.constants.Constants;
import frc.robot.subsystems.elevator.ElevatorConstants.Real;
import frc.robot.util.SparkUtil;
import java.util.function.DoubleSupplier;

public class ElevatorIOSpark implements ElevatorIO {
  private final SparkBase leaderSpark;
  private final SparkBase followerSpark;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;
  private final SparkMaxConfig leaderConfig;
  private final SparkMaxConfig followerConfig;
  private final SparkMaxConfig globalConfig;

  private final Debouncer connectedDebounce = new Debouncer(0.5);

  public ElevatorIOSpark() {
    leaderSpark = new SparkMax(leaderId, MotorType.kBrushless);
    followerSpark = new SparkMax(followerId, MotorType.kBrushless);

    encoder = leaderSpark.getEncoder();
    controller = leaderSpark.getClosedLoopController();

    globalConfig = new SparkMaxConfig();
    globalConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(motorCurrentLimit)
        .voltageCompensation(12);
    globalConfig
        .signals
        .appliedOutputPeriodMs((int) Constants.kLoopPeriodSecs * 1000)
        .busVoltagePeriodMs((int) Constants.kLoopPeriodSecs * 1000)
        .outputCurrentPeriodMs((int) Constants.kLoopPeriodSecs * 1000);

    leaderConfig = new SparkMaxConfig();
    leaderConfig.apply(globalConfig);
    leaderConfig
        .encoder
        .positionConversionFactor(encoderPositionFactor)
        .velocityConversionFactor(encoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    leaderConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(Real.kP, 0, Real.kD, 0);
    leaderConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20);

    SparkUtil.tryUntilOk(
        leaderSpark,
        5,
        () ->
            leaderSpark.configure(
                leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    followerConfig = new SparkMaxConfig();
    followerConfig.apply(globalConfig).follow(leaderSpark);

    SparkUtil.tryUntilOk(
        followerSpark,
        5,
        () ->
            followerSpark.configure(
                followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    SparkUtil.ifOk(leaderSpark, encoder::getPosition, (position) -> inputs.positionRad = position);

    SparkUtil.ifOk(leaderSpark, encoder::getVelocity, (vel) -> inputs.velocityRadPerSec = vel);

    SparkUtil.ifOk(
        leaderSpark,
        new DoubleSupplier[] {leaderSpark::getAppliedOutput, leaderSpark::getBusVoltage},
        (values) -> inputs.appliedVolts[0] = values[0] * values[1]);

    SparkUtil.ifOk(
        followerSpark,
        new DoubleSupplier[] {followerSpark::getAppliedOutput, followerSpark::getBusVoltage},
        (values) -> inputs.appliedVolts[1] = values[0] * values[1]);

    SparkUtil.ifOk(
        leaderSpark, leaderSpark::getOutputCurrent, (current) -> inputs.currentAmps[0] = current);
    SparkUtil.ifOk(
        followerSpark,
        followerSpark::getOutputCurrent,
        (current) -> inputs.currentAmps[1] = current);

    inputs.connected = connectedDebounce.calculate(true);
  }

  @Override
  public void runOpenLoop(double output) {
    leaderSpark.setVoltage(output);
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
    leaderSpark.stopMotor();
  }
}
