package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.PoseEstimator;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.AllianceFlipUtil;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class DriveCommands {
  // Drive
  private static final double DEADBAND = 0.1;

  private static final LoggedNetworkNumber LINEAR_VELOCITY_SCALAR =
      new LoggedNetworkNumber("TeleopDrive/LinearVelocityScalar", 1.0);
  private static final LoggedNetworkNumber ANGULAR_VELOCITY_SCALAR =
      new LoggedNetworkNumber("TeleopDrive/AngularVelocityScalar", 0.7);

  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;

  // Characterization
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.85; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      DriveBase driveBase,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), DEADBAND);
          double y = MathUtil.applyDeadband(ySupplier.getAsDouble(), DEADBAND);
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          x = Math.copySign(Math.pow(x, 2), x);
          y = Math.copySign(Math.pow(y, 2), y);
          omega = Math.copySign(Math.pow(omega, 2), omega);

          // Generate robot relative speeds
          double linearVelocityScalar = LINEAR_VELOCITY_SCALAR.get();
          double angularVelocityScalar = ANGULAR_VELOCITY_SCALAR.get();

          var speeds =
              new ChassisSpeeds(
                  x * DriveConstants.maxLinearVelocityMetersPerSec * linearVelocityScalar,
                  y * DriveConstants.maxLinearVelocityMetersPerSec * linearVelocityScalar,
                  omega * DriveConstants.maxAngularVelocityRadPerSec * angularVelocityScalar);

          // Convert to field relative
          Rotation2d rotation = PoseEstimator.getInstance().getRotation();
          rotation = AllianceFlipUtil.apply(rotation);
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rotation);

          // Apply speeds
          driveBase.runVelocity(speeds);
        },
        driveBase);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      DriveBase driveBase,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              // Apply deadband
              double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), DEADBAND);
              double y = MathUtil.applyDeadband(ySupplier.getAsDouble(), DEADBAND);

              // Square rotation value for more precise control
              x = Math.copySign(Math.pow(x, 2), x);
              y = Math.copySign(Math.pow(y, 2), y);

              // Calculate angular speed
              Rotation2d rotation = PoseEstimator.getInstance().getRotation();
              double omega =
                  angleController.calculate(
                      rotation.getRadians(), rotationSupplier.get().getRadians());

              // Generate robot relative speeds
              double linearVelocityScalar = LINEAR_VELOCITY_SCALAR.get();
              var speeds =
                  new ChassisSpeeds(
                      x * DriveConstants.maxLinearVelocityMetersPerSec * linearVelocityScalar,
                      y * DriveConstants.maxLinearVelocityMetersPerSec * linearVelocityScalar,
                      omega);

              // Convert to field relative
              rotation = AllianceFlipUtil.apply(rotation);
              speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rotation);

              // Apply speeds
              driveBase.runVelocity(speeds);
            },
            driveBase)
        .beforeStarting(
            () -> angleController.reset(PoseEstimator.getInstance().getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(DriveBase drive) {
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
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

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
  public static Command wheelRadiusCharacterization(DriveBase drive) {
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
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getGyroRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getGyroRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
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

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
