// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.ElevatorMechanismVisualizer;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  // Tunable numbers
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG");
  private static final LoggedTunableNumber maxVelocityMetersPerSec =
      new LoggedTunableNumber("Elevator/MaxVelocityMetersPerSec", 2.0);
  private static final LoggedTunableNumber maxAccelerationMetersPerSec2 =
      new LoggedTunableNumber("Elevator/MaxAccelerationMetersPerSec2", 10);
  private static final LoggedTunableNumber tolerance =
      new LoggedTunableNumber("Elevator/Tolerance", 0.2);

  static {
    switch (Constants.getRobotType()) {
      case ROBOT_2025_COMP -> {
        kP.initDefault(ElevatorConstants.Real.kP);
        kD.initDefault(ElevatorConstants.Real.kD);
        kS.initDefault(ElevatorConstants.Real.kS);
        kG.initDefault(ElevatorConstants.Real.kG);
      }
      case ROBOT_SIMBOT -> {
        kP.initDefault(ElevatorConstants.Sim.kP);
        kD.initDefault(ElevatorConstants.Sim.kD);
        kS.initDefault(ElevatorConstants.Sim.kS);
        kG.initDefault(ElevatorConstants.Sim.kG);
      }
    }
  }

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final Alert motorDisconnectedAlert =
      new Alert("Elevator motor disconnected!", Alert.AlertType.kWarning);
  private BooleanSupplier coastOverride = () -> false;
  private BooleanSupplier disabledOverride = () -> false;

  @AutoLogOutput private boolean brakeModeEnabled = true;

  private TrapezoidProfile profile;
  @Getter private State setpoint = new State();
  private Supplier<State> goal = State::new;
  private boolean stopProfile = false;
  @Getter private boolean shouldEStop = false;
  @Setter private boolean isEStopped = false;

  @AutoLogOutput(key = "Elevator/HomedPositionRad")
  private double homedPosition = 0.0;

  @AutoLogOutput @Getter private boolean homed = false;

  private Debouncer toleranceDebouncer = new Debouncer(0.25, DebounceType.kRising);

  @Getter
  @AutoLogOutput(key = "Elevator/Profile/AtGoal")
  private boolean atGoal = false;

  @Setter private boolean stowed = false;

  @Getter
  @AutoLogOutput(key = "Elevator/Profile/AtSetpoint")
  private boolean atSetpoint = false;

  private final ElevatorMechanismVisualizer setpointVisualizer =
      new ElevatorMechanismVisualizer(
          "Elevator",
          "Setpoint",
          ElevatorConstants.minElevatorHeightMeters,
          ElevatorConstants.elevatorExtensionOriginPose3d,
          2,
          2);

  private final ElevatorMechanismVisualizer measuredVisualizer =
      new ElevatorMechanismVisualizer(
          "Elevator",
          "Measured",
          ElevatorConstants.minElevatorHeightMeters,
          ElevatorConstants.elevatorExtensionOriginPose3d,
          2,
          2);

  public Elevator(ElevatorIO io) {
    this.io = io;

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                maxVelocityMetersPerSec.get(), maxAccelerationMetersPerSec2.get()));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    motorDisconnectedAlert.set(!inputs.connected);

    // Update tunable numbers
    if (maxVelocityMetersPerSec.hasChanged(hashCode())
        || maxAccelerationMetersPerSec2.hasChanged(hashCode())) {
      profile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  maxVelocityMetersPerSec.get(), maxAccelerationMetersPerSec2.get()));
    }

    // Run profile
    final boolean shouldRunProfile =
        !stopProfile
            && !coastOverride.getAsBoolean()
            && !disabledOverride.getAsBoolean()
            // && homed
            && !isEStopped;
    Logger.recordOutput("Elevator/RunningProfile", shouldRunProfile);
    // Check if out of tolerance
    boolean outOfTolerance = Math.abs(getPositionMeters() - setpoint.position) > tolerance.get();
    shouldEStop = toleranceDebouncer.calculate(outOfTolerance && shouldRunProfile);

    if (shouldRunProfile) {
      // Clamp goal
      var goalState =
          new State(
              MathUtil.clamp(goal.get().position, 0.0, ElevatorConstants.maxElevatorHeightMeters),
              goal.get().velocity);
      setpoint = profile.calculate(Constants.kLoopPeriodSecs, setpoint, goalState);
      if (setpoint.position < 0.0
          || setpoint.position > ElevatorConstants.maxElevatorHeightMeters) {
        setpoint =
            new State(
                MathUtil.clamp(setpoint.position, 0.0, ElevatorConstants.maxElevatorHeightMeters),
                0.0);
      }
      io.runPosition(
          setpoint.position / ElevatorConstants.drumRadius + homedPosition,
          kS.get() * Math.signum(setpoint.velocity) // Magnitude irrelevant
              + kG.get() * Math.sin(ElevatorConstants.elevatorAngle));
      // Check at goal
      atGoal =
          EqualsUtil.epsilonEquals(setpoint.position, goalState.position, 0.01)
              && EqualsUtil.epsilonEquals(setpoint.velocity, goalState.velocity, 0.01);

      atSetpoint =
          EqualsUtil.epsilonEquals(
                  setpoint.position, inputs.positionRad * ElevatorConstants.drumRadius, 0.01)
              && EqualsUtil.epsilonEquals(
                  setpoint.velocity,
                  inputs.velocityRadPerSec * ElevatorConstants.drumRadius,
                  0.01);

      // Stop running elevator down when in stow
      if (stowed && atGoal) {
        io.stop();
      }

      // Log state
      Logger.recordOutput("Elevator/Profile/SetpointPositionMeters", setpoint.position);
      Logger.recordOutput("Elevator/Profile/SetpointVelocityMetersPerSec", setpoint.velocity);
      Logger.recordOutput("Elevator/Profile/GoalPositionMeters", goalState.position);
      Logger.recordOutput("Elevator/Profile/GoalVelocityMetersPerSec", goalState.velocity);
      Logger.recordOutput(
          "Elevator/Profile/MeasuredPositionMeters",
          inputs.positionRad * ElevatorConstants.drumRadius);
      Logger.recordOutput(
          "Elevator/Profile/MeasuredVelocityMetersPerSec",
          inputs.velocityRadPerSec * ElevatorConstants.drumRadius);

    } else {
      // Reset setpoint
      setpoint = new State(getPositionMeters(), 0.0);

      // Clear logs
      Logger.recordOutput("Elevator/Profile/SetpointPositionMeters", 0.0);
      Logger.recordOutput("Elevator/Profile/SetpointVelocityMetersPerSec", 0.0);
      Logger.recordOutput("Elevator/Profile/GoalPositionMeters", 0.0);
      Logger.recordOutput("Elevator/Profile/GoalVelocityMetersPerSec", 0.0);
    }
    if (isEStopped) {
      io.stop();
    }

    if (DriverStation.isDisabled()) {
      setGoal(() -> 0.0);
    }

    // Log state
    Logger.recordOutput("Elevator/CoastOverride", coastOverride.getAsBoolean());
    Logger.recordOutput("Elevator/DisabledOverride", disabledOverride.getAsBoolean());
    Logger.recordOutput(
        "Elevator/MeasuredVelocityMetersPerSec",
        inputs.velocityRadPerSec * ElevatorConstants.drumRadius);

    setpointVisualizer.update(setpoint.position);
    measuredVisualizer.update(inputs.positionRad * ElevatorConstants.drumRadius);
  }

  public void setGoal(DoubleSupplier goal) {
    setGoal(() -> new State(goal.getAsDouble(), 0.0));
  }

  public void setGoal(Supplier<State> goal) {
    atGoal = false;
    this.goal = goal;
  }

  public void setOverrides(BooleanSupplier coastOverride, BooleanSupplier disabledOverride) {
    this.coastOverride = coastOverride;
    this.disabledOverride = disabledOverride;
  }

  /** Get position of elevator in meters with 0 at home */
  @AutoLogOutput(key = "Elevator/MeasuredHeightMeters")
  public double getPositionMeters() {
    return (inputs.positionRad - homedPosition) * ElevatorConstants.drumRadius;
  }

  public double getGoalMeters() {
    return goal.get().position;
  }
}
