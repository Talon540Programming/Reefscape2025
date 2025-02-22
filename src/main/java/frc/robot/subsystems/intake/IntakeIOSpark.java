package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;

public class IntakeIOSpark implements IntakeIO {
  private final SparkBase spark;
  private final RelativeEncoder encoder;

  private final Debouncer connectedDebouncer = new Debouncer(.5);

  public IntakeIOSpark() {
    spark = new SparkMax(11, SparkLowLevel.MotorType.kBrushless);
    encoder = spark.getEncoder();

    var config = new SparkMaxConfig();
    config
        .inverted(inverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40, 50)
        .voltageCompensation(12.0);

    config
        .encoder
        .positionConversionFactor(positionConversionFactor)
        .velocityConversionFactor(velocityConversionFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    config
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    spark.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.positionRads = encoder.getPosition();
    inputs.velocityRadsPerSec = encoder.getVelocity();
    inputs.appliedVoltage = spark.getAppliedOutput() * spark.getBusVoltage();
    inputs.currentAmps = spark.getOutputCurrent();
    inputs.tempCelsius = spark.getMotorTemperature();

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
    brakeModeConfig.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);

    spark.configure(
        brakeModeConfig,
        SparkBase.ResetMode.kNoResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);
  }
}
