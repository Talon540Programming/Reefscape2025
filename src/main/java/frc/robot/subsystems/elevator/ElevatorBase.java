package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.Debouncer;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ElevatorBase extends SubsystemBase {
  // Tunable numbers
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA");

  private static final LoggedTunableNumber maxVelocityMetersPerSec =
      new LoggedTunableNumber("Elevator/MaxVelocityMetersPerSec", 2.0);
  private static final LoggedTunableNumber maxAccelerationMetersPerSec2 =
      new LoggedTunableNumber("Elevator/MaxAccelerationMetersPerSec2", 10);

  private static final LoggedTunableNumber homingVolts =
      new LoggedTunableNumber("Elevator/HomingVolts", -2.0);
  private static final LoggedTunableNumber homingTimeSecs =
      new LoggedTunableNumber("Elevator/HomingTimeSecs", 0.25);
  private static final LoggedTunableNumber homingVelocityThresh =
      new LoggedTunableNumber("Elevator/HomingVelocityThresh", 5.0);

  private static final LoggedTunableNumber toleranceMeters =
      new LoggedTunableNumber("Elevator/ToleranceMeters", 0.2);

  static {
    switch (Constants.getRobot()) {
      case COMPBOT -> {
        kP.initDefault(0.3);
        kD.initDefault(0.25);
        kS.initDefault(0);
        kG.initDefault(1.05);
        kA.initDefault(0.0);
      }
      case SIMBOT -> {
        kP.initDefault(0); // TODO
        kD.initDefault(0); // TODO
        kS.initDefault(0); // TODO
        kG.initDefault(0); // TODO
        kA.initDefault(0); // TODO
      }
    }
  }

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final Alert motorDisconnectedAlert =
      new Alert("Elevator leader motor disconnected!", Alert.AlertType.kWarning);
  private final Alert followerDisconnectedAlert =
      new Alert("Elevator follower motor disconnected!", Alert.AlertType.kWarning);

  @AutoLogOutput(key = "Elevator/Goal")
  @Getter
  @Setter
  private ElevatorState goal = ElevatorState.START;

  private TrapezoidProfile profile;
  private State setpoint = new State();

  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0.0, 0.0, 0.0);

  private boolean profileDisabled = false;
  @Setter private boolean eStopped = false;

  @AutoLogOutput(key = "Elevator/Homed")
  @Getter
  private boolean homed = false;

  private final Debouncer toleranceDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kRising);
  private final Alert outOfTolleranceAlert =
      new Alert(
          "Elevator emergency disabled due to high position error. Rehome the elevator to reset.",
          Alert.AlertType.kWarning);

  @AutoLogOutput(key = "Elevator/Profile/AtGoal")
  @Getter
  private boolean atGoal = false;

  private final ElevatorVisualizer measuredVisualizer = new ElevatorVisualizer("Measured");
  private final ElevatorVisualizer setpointVisualizer = new ElevatorVisualizer("Setpoint");

  private final SysIdRoutine sysId;

  public ElevatorBase(ElevatorIO io) {
    this.io = io;

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                maxVelocityMetersPerSec.get(), maxAccelerationMetersPerSec2.get()));

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(.1).per(Second),
                Volts.of(4),
                Seconds.of(
                    45), // Effectively disable the timeout and allow the Command factories to set
                // them
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.runOpenLoop(voltage.in(Volts)),
                null, // No log consumer, since data is recorded by AdvantageKit
                this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    motorDisconnectedAlert.set(!inputs.leaderConnected);
    followerDisconnectedAlert.set(!inputs.followerConnected);

    // Update tunable numbers
    LoggedTunableNumber.ifChanged(() -> io.setPID(kP.get(), 0.0, kD.get()), true, kP, kD);
    LoggedTunableNumber.ifChanged(
        () -> {
          feedforward.setKs(kS.get());
          feedforward.setKg(kG.get());
          feedforward.setKa(kA.get());
        },
        true,
        kS,
        kG,
        kA);


    LoggedTunableNumber.ifChanged(
        () ->
            profile =
                new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(
                        maxVelocityMetersPerSec.get(), maxAccelerationMetersPerSec2.get())),
        true,
        maxVelocityMetersPerSec,
        maxAccelerationMetersPerSec2);

    // Run profile
    final boolean shouldRunProfile =
        !profileDisabled && homed && !eStopped && DriverStation.isEnabled();
    Logger.recordOutput("Elevator/RunningProfile", shouldRunProfile);

    // // Check if out of tolerance
    // boolean outOfTolerance =
    //     Math.abs(getPositionMeters() - setpoint.position) > toleranceMeters.get();
    // boolean shouldEStop = toleranceDebouncer.calculate(outOfTolerance && shouldRunProfile);
    //
    // outOfTolleranceAlert.set(shouldEStop);
    // if (shouldEStop) {
    //   eStopped = true;
    // }

    if (shouldRunProfile) {
      //   Clamp goal
      var goalState =
          new State(
              MathUtil.clamp(goal.getElevatorHeightMeters().getAsDouble(), 0.0, maxTravel), 0);

      double previousVelocity = setpoint.velocity;
      setpoint = profile.calculate(Constants.kLoopPeriodSecs, setpoint, goalState);
      if (setpoint.position < 0.0 || setpoint.position > maxTravel) {
        setpoint = new State(MathUtil.clamp(setpoint.position, 0.0, maxTravel), 0.0);
      }

      io.runPosition(
          setpoint.position / drumRadius / numStages,
          feedforward.calculateWithVelocities(setpoint.velocity, previousVelocity));

      // Check at goal
      atGoal =
          EqualsUtil.epsilonEquals(setpoint.position, goalState.position)
              && EqualsUtil.epsilonEquals(setpoint.velocity, goalState.velocity);

      // Stop if at the bottom
      if (atGoal && EqualsUtil.epsilonEquals(setpoint.position, 0.0)) {
        io.stop();
      }

      // Log state
      Logger.recordOutput("Elevator/Profile/SetpointPositionMeters", setpoint.position);
      Logger.recordOutput("Elevator/Profile/SetpointVelocityMetersPerSec", setpoint.velocity);
      Logger.recordOutput("Elevator/Profile/GoalPositionMeters", goalState.position);
      Logger.recordOutput("Elevator/Profile/GoalVelocityMetersPerSec", goalState.velocity);
    } else {
      if (DriverStation.isDisabled()) {
        goal = ElevatorState.STOW;
      }

      // Reset setpoint
      setpoint = new State(getPositionMeters(), 0.0);

      // Clear logs
      Logger.recordOutput("Elevator/Profile/SetpointPositionMeters", 0.0);
      Logger.recordOutput("Elevator/Profile/SetpointVelocityMetersPerSec", 0.0);
      Logger.recordOutput("Elevator/Profile/GoalPositionMeters", 0.0);
      Logger.recordOutput("Elevator/Profile/GoalVelocityMetersPerSec", 0.0);
    }

    if (eStopped) {
      io.stop();
    }

    Logger.recordOutput(
        "Elevator/MeasuredVelocityMetersPerSec", inputs.velocityRadPerSec * drumRadius);

    // If not homed, schedule that command
    if (!homed && !profileDisabled) {
      homingSequence().schedule();
    }

    measuredVisualizer.update(getPositionMeters());
    setpointVisualizer.update(setpoint.position);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction, double timeoutSecs) {
    return runOnce(
            () -> {
              profileDisabled = true;
              io.stop();
            })
        .andThen(Commands.waitSeconds(1))
        .andThen(sysId.quasistatic(direction).withTimeout(timeoutSecs))
        .finallyDo(() -> profileDisabled = false);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction, double timeoutSecs) {
    return runOnce(
            () -> {
              profileDisabled = true;
              io.stop();
            })
        .andThen(Commands.waitSeconds(1))
        .andThen(sysId.dynamic(direction).withTimeout(timeoutSecs))
        .finallyDo(() -> profileDisabled = false);
  }

  public Command homingSequence() {
    var homingDebouncer = new Debouncer(homingTimeSecs.get());
    return Commands.startRun(
            () -> {
              profileDisabled = true;
              homed = false;
              homingDebouncer.calculate(false);
            },
            () -> {
              io.runOpenLoop(homingVolts.get());
              homed =
                  homingDebouncer.calculate(
                      Math.abs(inputs.velocityRadPerSec) <= homingVelocityThresh.get());
            })
        .until(() -> homed)
        .andThen(
            () -> {
              io.resetOrigin();
              homed = true;
            })
        .finallyDo(() -> profileDisabled = false);
  }

  @AutoLogOutput(key = "Elevator/MeasuredHeightMeters")
  public double getPositionMeters() {
    return inputs.positionRads * drumRadius * numStages;
  }

  public double getGoalMeters() {
    return goal.getElevatorHeightMeters().getAsDouble();
  }
}
