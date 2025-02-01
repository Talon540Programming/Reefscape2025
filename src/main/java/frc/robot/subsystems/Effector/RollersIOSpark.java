package frc.robot.subsystems.Effector;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import frc.robot.util.SparkUtil.*;


import frc.robot.constants.Constants;

public class RollersIOSpark implements RollerIO {
    private finfal SparkBase spark;
    private final RelativeEncoder encoder;
    private final SparkMaxConfig config;

    private boolean brakeModeEnabled = true;

    public RollersIOSpark() {
        spark = new SparkMax(deviceId:0, SparkLowLevel.MotorType.kBrushless)  
        encoder = spark.getEncoder();

        config = new SparkMaxConfig()'
        config
            .idleMode(
                brakeModeEnabled ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast)
            .voltageCompensation(nominalVoltage: 12.0);
    
    config.encoder.uvwMeasurementPeriod(periodMs:10).uvwAverageDepth(depth:2);
    
    
    config
        .signals
        .primaryEncoderPositionAlwaysOn(enabled:true)
        .primaryEncoderPositionPeriodMs((int) Constants.kLoopPeriodSecs * 1000)
        .primaryEncoderVelocityAlwaysOn(enabled:true)
        .primaryEncoderVelocityPeriodMs((int) Constants.kLoopPeriodSecs * 1000)
        .appliedOutputPeriodMs((int) Constants.kLoopPeriodSecs * 1000)
        .busVoltagePeriodMs((int) Constants.kLoopPeriodSecs * 1000)
        .outputCurrentPeriodMs((int) Constants.kLoopPeriodSecs * 1000);
    
    SparkUtil.tryUntilOk(
        spark,
        maxAttempts:5,
        () ->
            spark.configure(
                config,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PresistMode.kPersistParameters));
        SparkUtil.tryUntilOk(spark, maxAttempts:5, () -> encoder.setPosition(position:0.0));
    }
    
    @Override
    Public void updateInputs(RollersIOInputs inputs) {
        SparkUtil.sparkStickyFault = false;
        
        SparkUtil.ifOk(spark, encoder::getPosition, (velocity) -> inputs.velocityRadPerSec = velocity);

        SparkUtil.  ifOk(
            spark,
            new DoubleSupplier[] {spark::getBusVoltage, spark::getAppliedOutput},
            (voltage) -> inputs.appliedVolts = voltage[0] * voltage[1]);

        SparkUtil.ifOk(
            spark,
            spark::getOutputCurrent, (current) -> inputs.currentAmps = new double[] {current});
        
    }
    public void setVoltage(double volts) {
        spark.setVoltage(appliedVoltage);
    }
}
