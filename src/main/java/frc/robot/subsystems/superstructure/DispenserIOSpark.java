package frc.robot.subsystems.superstructure;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.Dispenser.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.Debouncer;

public class DispenserIOSpark implements DispenserIO {
  private final SparkBase spark;
  private final RelativeEncoder encoder;

  private final Debouncer connectedDebouncer = new Debouncer(.5);

  // End Dispenser beam break
  private final DigitalInput rearBeamBreak = new DigitalInput(0);

  public DispenserIOSpark() {
    spark = new SparkMax(14, SparkLowLevel.MotorType.kBrushless);
    encoder = spark.getEncoder();

    var config = new SparkMaxConfig();
    config
        .inverted(inverted)
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .smartCurrentLimit(40, 50)
        .voltageCompensation(12.0);

    config
        .encoder
        .positionConversionFactor(2 * Math.PI / gearing)
        .velocityConversionFactor(2 * Math.PI / 60.0 / gearing)
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
        .outputCurrentPeriodMs(20)
        .motorTemperaturePeriodMs(20);

    spark.configure(
        config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(DispenserIOInputs inputs) {
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
