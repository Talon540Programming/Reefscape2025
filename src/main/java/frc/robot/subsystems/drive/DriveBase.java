package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.PoseEstimator;
import frc.robot.PoseEstimator.OdometryObservation;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.swerve.SwerveSetpointGenerator;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Queue;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DriveBase extends SubsystemBase {
  // Characterization
  private static final double FF_START_DELAY = 2; // Secs
  private static final double FF_RAMP_RATE = 3.5; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged m_gyroInputs = new GyroIOInputsAutoLogged();

  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  private final Queue<Double> m_timestampsQueue;
  private final OdometryTimestampsInputAutoLogged m_timestampInputs =
      new OdometryTimestampsInputAutoLogged();
  private final Alert gyroDisconnectedAlert =
      new Alert(
          "Disconnected gyro, using kinematic approximation as fallback.", Alert.AlertType.kError);

  private static final LoggedTunableNumber coastWaitTime =
      new LoggedTunableNumber("Drive/CoastWaitTimeSeconds", 0.5);
  private static final LoggedTunableNumber coastMetersPerSecondThreshold =
      new LoggedTunableNumber("Drive/CoastMetersPerSecThreshold", .05);

  private final Timer lastMovementTimer = new Timer();

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(DriveConstants.moduleTranslations);

  private SwerveSetpointGenerator.SwerveSetpoint currentSetpoint =
      new SwerveSetpointGenerator.SwerveSetpoint(
          new ChassisSpeeds(),
          new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
          });
  private final SwerveSetpointGenerator swerveSetpointGenerator;

  @AutoLogOutput(key = "Drive/ClosedLoopMode")
  private boolean CLOSED_LOOP_MODE = false;

  @AutoLogOutput(key = "Drive/BrakeModeEnabled")
  private boolean BRAKE_MODE = true;

  private final SysIdRoutine sysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              null,
              null,
              (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
          new SysIdRoutine.Mechanism(
              (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

  public DriveBase(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    swerveSetpointGenerator =
        new SwerveSetpointGenerator(kinematics, DriveConstants.moduleTranslations);

    m_timestampsQueue = OdometryManager.getInstance().getTimestampQueue();

    // Start odometry thread
    OdometryManager.getInstance().start();

    lastMovementTimer.start();
    setBrakeMode(true);
  }

  @Override
  public void periodic() {
    OdometryManager.odometryLock.lock();
    try {
      // Update and log gyro Inputs
      gyroIO.updateInputs(m_gyroInputs);
      Logger.processInputs("Drive/Gyro", m_gyroInputs);
      // Update and log only on modules
      for (var module : modules) {
        module.updateInputs();
      }
      // Get current sample timestamps
      m_timestampInputs.timestamps =
          m_timestampsQueue.stream().mapToDouble((Double value) -> value).toArray();
      m_timestampsQueue.clear();
      Logger.processInputs("Drive/OdometryTimestamps", m_timestampInputs);
    } finally {
      OdometryManager.odometryLock.unlock();
    }

    // Call periodic on modules
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsUnoptimized", new SwerveModuleState[] {});
    }

    var timestamps = m_timestampInputs.timestamps;
    var sampleCount = timestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
      for (int j = 0; j < 4; j++) {
        wheelPositions[j] = modules[j].getOdometryPositions()[i];
      }
      PoseEstimator.getInstance()
          .addOdometryObservation(
              new OdometryObservation(
                  wheelPositions,
                  Optional.ofNullable(
                      m_gyroInputs.connected ? m_gyroInputs.odometryYawPositions[i] : null),
                  timestamps[i]));
    }

    // Disable brake mode a short duration after the robot is disabled
    for (var module : modules) {
      if (Math.abs(module.getVelocityMetersPerSec()) > coastMetersPerSecondThreshold.get()) {
        lastMovementTimer.reset();
        break;
      }
    }

    if (DriverStation.isEnabled()) {
      setBrakeMode(true);
    } else if (lastMovementTimer.hasElapsed(coastWaitTime.get())) {
      setBrakeMode(false);
    }

    // Update current setpoint if not in velocity mode
    if (!CLOSED_LOOP_MODE) {
      currentSetpoint =
          new SwerveSetpointGenerator.SwerveSetpoint(getChassisSpeeds(), getModuleStates());
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!m_gyroInputs.connected && Constants.getMode() != Constants.Mode.SIM);
  }

  /** Set brake mode to {@code enabled} doesn't change brake mode if already set. */
  private void setBrakeMode(boolean enabled) {
    if (BRAKE_MODE != enabled) {
      for (var module : modules) {
        module.setDriveBrakeMode(enabled);
      }
    }
    BRAKE_MODE = enabled;
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    CLOSED_LOOP_MODE = true;

    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.kLoopPeriodSecs);
    SwerveModuleState[] setpointStatesUnoptimized = kinematics.toSwerveModuleStates(discreteSpeeds);
    currentSetpoint =
        swerveSetpointGenerator.generateSetpoint(
            DriveConstants.moduleLimitsFree,
            currentSetpoint,
            discreteSpeeds,
            Constants.kLoopPeriodSecs);
    SwerveModuleState[] setpointStates = currentSetpoint.moduleStates();

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/SetpointsUnoptimized", setpointStatesUnoptimized);
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", currentSetpoint.chassisSpeeds());
    Logger.recordOutput(
        "SwerveChassisSpeeds/SetpointsUnoptimized", currentSetpoint.chassisSpeeds());

    SwerveModuleState[] measuredStates = getModuleStates();
    SwerveModuleState[] stateErrors = new SwerveModuleState[4];

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);

      stateErrors[i] =
          new SwerveModuleState(
              setpointStates[i].speedMetersPerSecond - measuredStates[i].speedMetersPerSecond,
              setpointStates[i].angle.minus(measuredStates[i].angle));
    }

    Logger.recordOutput("SwerveStates/Error", stateErrors);
  }

  // CAN CONNECTOR ON CANID 6 MUST BE REPLACED BEFORE COMP

  /** Runs the drive in a straight(ish) line with the specified drive output. */
  public void runCharacterization(double output) {
    CLOSED_LOOP_MODE = false;

    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = DriveConstants.moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns the module states (turn angles and drive velocities) for all the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  // @AutoLogOutput(key = "SwerveStates/Error")
  // private SwerveModuleState[] getStateError() {
  //   SwerveModuleState[] states = getModuleStates();

  //   SwerveModuleState[] errors = new SwerveModuleState[4];
  //   for (int i = 0; i < 4; i++) {
  //     errors[i] = states[i] -
  //   }
  // }

  /** Returns the module positions (turn angles and drive positions) for all the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rad/sec. */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the raw gyro rotation read by the IMU */
  public Rotation2d getGyroRotation() {
    return m_gyroInputs.yawPosition;
  }

  public Command runAtVolts(double volts) {
    return Commands.run(
        () -> {
          // System.out.println(volts);
          for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(volts);
          }
        });
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public Command feedforwardCharacterization() {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  this.runCharacterization(0.0);
                },
                this)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  this.runCharacterization(voltage);
                  velocitySamples.add(this.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                this)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  SmartDashboard.putString("kS", formatter.format(kS));
                  SmartDashboard.putString("kV", formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public Command wheelRadiusCharacterization() {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  this.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                this)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = this.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = this.getGyroRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = this.getGyroRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = this.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");

                      SmartDashboard.putString(
                          "Wheel Delta", formatter.format(wheelDelta) + " radians");
                      SmartDashboard.putString(
                          "Gyro Delta", formatter.format(state.gyroDelta) + " radians");
                      SmartDashboard.putString(
                          "Wheel Radius",
                          formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }

  public static double getMaxLinearVelocityMetersPerSecond() {
    return DriveConstants.maxLinearVelocityMetersPerSec;
  }

  public static double getMaxAngularVelocityRadPerSec() {
    return DriveConstants.maxAngularVelocityRadPerSec;
  }

  public static Translation2d[] getModuleTranslations() {
    return DriveConstants.moduleTranslations;
  }
}
