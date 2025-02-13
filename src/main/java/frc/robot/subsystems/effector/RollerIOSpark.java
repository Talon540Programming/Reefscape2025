package frc.robot.subsystems.effector;

import static frc.robot.subsystems.effector.EffectorConstants.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.subsystems.effector.IndexerIO.IndexerIOInputs;
import frc.robot.util.SparkUtil;

public class RollerIOSpark implements RollerIO {
  private final SparkBase spark;
  private final RelativeEncoder encoder;
  private final SparkMaxConfig config;

  private final DigitalInput beamBreak;

  private final Debouncer connectedDebounce = new Debouncer(0.5);

  public RollerIOSpark() {
    beamBreak = new DigitalInput(beamBreakId);

    spark = new SparkMax(rollerId, MotorType.kBrushless);
    encoder = spark.getEncoder();

    config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);

    config
        .signals
        .appliedOutputPeriodMs((int) Constants.kLoopPeriodSecs * 1000)
        .busVoltagePeriodMs((int) Constants.kLoopPeriodSecs * 1000)
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20);

    config
        .encoder
        .positionConversionFactor(rollerPositionConversion)
        .velocityConversionFactor(rollerVelocityConversion)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    SparkUtil.tryUntilOk(
        spark,
        5,
        () ->
            spark.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    SparkUtil.ifOk(spark, encoder::getVelocity, (velocity) -> inputs.velocityRadPerSec = velocity);
    SparkUtil.ifOk(
        spark,
        new DoubleSupplier[] {spark::getAppliedOutput, spark::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    SparkUtil.ifOk(spark, spark::getOutputCurrent, (current) -> inputs.currentAmps = current);

    inputs.connected = connectedDebounce.calculate(true);
    inputs.hasCoral = !beamBreak.get();
  }

  @Override
  public void runOpenLoop(double output) {
    spark.setVoltage(output);
  }

  @Override
  public void stop() {
    spark.setVoltage(0);
  }
}
