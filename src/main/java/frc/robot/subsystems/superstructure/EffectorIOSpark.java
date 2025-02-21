package frc.robot.subsystems.superstructure;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;

public class EffectorIOSpark implements EffectorIO {
  private final SparkBase spark;
  private final RelativeEncoder encoder;

  private final Debouncer connectedDebouncer = new Debouncer(.5);

  // End Effector beam break
  private final DigitalInput rearBeamBreak = new DigitalInput(0);

  public EffectorIOSpark() {
    spark = new SparkMax(12, SparkLowLevel.MotorType.kBrushless);
    encoder = spark.getEncoder();

    var config = new SparkMaxConfig();
    config
        .inverted(effectorInverted)
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .smartCurrentLimit(40, 50)
        .voltageCompensation(12.0);

    config
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    config
        .encoder
        .positionConversionFactor(effectorPositionConversionFactor)
        .velocityConversionFactor(effectorVelocityConversionFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    spark.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(EffectorIOInputs inputs) {
    inputs.positionRads = encoder.getPosition();
    inputs.velocityRadsPerSec = encoder.getVelocity();
    inputs.appliedVoltage = spark.getAppliedOutput() * spark.getBusVoltage();
    inputs.currentAmps = spark.getOutputCurrent();
    inputs.tempCelsius = spark.getMotorTemperature();

    inputs.rearBeamBreakBroken = !rearBeamBreak.get();

    inputs.connected = connectedDebouncer.calculate(!spark.hasActiveFault());
  }

  @Override
  public void runVolts(double output) {
    spark.setVoltage(output);
  }

  @Override
  public void stop() {
    spark.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    var brakeModeConfig = new SparkMaxConfig();
    brakeModeConfig.idleMode(
        enabled ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);

    spark.configure(
        brakeModeConfig,
        SparkBase.ResetMode.kNoResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);
  }
}
