package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants;
import frc.robot.util.Debouncer;
import java.util.Queue;

public class ModuleIOSpark implements ModuleIO {
  private final ModuleConfig config;

  // Hardware objects
  private final SparkMax driveSpark;
  private final SparkMax turnSpark;
  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnEncoder;
  private final AnalogEncoder turnAbsoluteEncoder;

  // Closed loop controllers
  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;
  private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0, 0);

  // Queue inputs from odometry thread
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

  public ModuleIOSpark(int index) {
    switch (Constants.getRobot()) {
      case COMPBOT -> {
        config = moduleConfigs[index];
      }
      default ->
          throw new IllegalStateException(
              "Unexpected RobotType for Spark Module: " + Constants.getRobot());
    }

    // Initialize Hardware Devices
    driveSpark = new SparkMax(config.driveMotorId(), MotorType.kBrushless);
    turnSpark = new SparkMax(config.turnMotorId(), MotorType.kBrushless);

    driveEncoder = driveSpark.getEncoder();
    turnEncoder = turnSpark.getEncoder();
    turnAbsoluteEncoder = new AnalogEncoder(config.encoderChannel(), 2 * Math.PI, 0);

    driveController = driveSpark.getClosedLoopController();
    turnController = turnSpark.getClosedLoopController();

    // Configure Drive
    var driveConfig = new SparkMaxConfig();
    driveConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).voltageCompensation(12.0);
    driveConfig
        .encoder
        .positionConversionFactor(2 * Math.PI / config.driveGearing())
        .velocityConversionFactor((2 * Math.PI) / 60.0 / config.driveGearing())
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    driveConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(0.0, 0.0, 0.0, 0.0);
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequencyHz))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20)
        .motorTemperaturePeriodMs(20);

    driveSpark.configure(
        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    driveEncoder.setPosition(0.0);

    // Configure Turn
    var turnConfig = new SparkMaxConfig();
    turnConfig
        .idleMode(IdleMode.kBrake)
        .inverted(config.turnInverted())
        .smartCurrentLimit(20)
        .voltageCompensation(12.0);
    turnConfig
        .encoder
        .positionConversionFactor(2 * Math.PI / config.turnGearing())
        .velocityConversionFactor((2 * Math.PI) / 60.0 / config.turnGearing())
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(-Math.PI, Math.PI)
        .pidf(0.0, 0.0, 0.0, 0.0);
    turnConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequencyHz))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20)
        .motorTemperaturePeriodMs(20);

    turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // We run PID on the controller, so its requires to tell it what the current true position is
    // based on the AbsoluteEncoder
    turnEncoder.setPosition(getOffsetAbsoluteAngle().getRadians());

    drivePositionQueue = OdometryManager.getInstance().registerSignal(driveEncoder::getPosition);
    turnPositionQueue = OdometryManager.getInstance().registerSignal(turnEncoder::getPosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad = driveEncoder.getPosition();
    inputs.driveVelocityRadPerSec = driveEncoder.getVelocity();
    inputs.driveAppliedVolts = driveSpark.getAppliedOutput() * driveSpark.getBusVoltage();
    inputs.driveCurrentAmps = driveSpark.getOutputCurrent();
    inputs.driveTempCelsius = driveSpark.getMotorTemperature();

    inputs.turnAbsolutePosition = getOffsetAbsoluteAngle();
    inputs.turnPosition = Rotation2d.fromRadians(turnEncoder.getPosition());
    inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
    inputs.turnAppliedVolts = turnSpark.getAppliedOutput() * turnSpark.getBusVoltage();
    inputs.turnCurrentAmps = turnSpark.getOutputCurrent();
    inputs.turnTempCelsius = turnSpark.getMotorTemperature();

    inputs.driveConnected = driveConnectedDebounce.calculate(!driveSpark.hasActiveFault());
    inputs.turnConnected = turnConnectedDebounce.calculate(!turnSpark.hasActiveFault());

    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    drivePositionQueue.clear();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream().map(Rotation2d::fromRadians).toArray(Rotation2d[]::new);
    turnPositionQueue.clear();
  }

  @Override
  public void runDriveOpenLoop(double output) {
    driveSpark.setVoltage(output);
  }

  @Override
  public void runTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  @Override
  public void runDriveVelocity(double velocityRadPerSec) {
    double ffVolts = driveFeedforward.calculate(velocityRadPerSec);
    driveController.setReference(
        velocityRadPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void runTurnPosition(Rotation2d rotation) {
    // Because internal encoder is relative, we want to wrap to the range -π to π radians.
    var updatedSetpoint = MathUtil.angleModulus(rotation.getRadians());
    turnController.setReference(updatedSetpoint, ControlType.kPosition);
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    var drivePIDConfig = new SparkMaxConfig();
    drivePIDConfig.closedLoop.pid(kP, kI, kD);

    driveSpark.configure(
        drivePIDConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setDriveFF(double kS, double kV) {
    driveFeedforward.setKs(kS);
    driveFeedforward.setKv(kV);
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    var turnPIDConfig = new SparkMaxConfig();
    turnPIDConfig.closedLoop.pid(kP, kI, kD);

    turnSpark.configure(
        turnPIDConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void setDriveBrakeMode(boolean enabled) {
    var brakeModeConfig = new SparkMaxConfig();
    brakeModeConfig.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);

    driveSpark.configure(
        brakeModeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private Rotation2d getOffsetAbsoluteAngle() {
    return Rotation2d.fromRadians(turnAbsoluteEncoder.get()).minus(config.encoderOffset());
  }
}
