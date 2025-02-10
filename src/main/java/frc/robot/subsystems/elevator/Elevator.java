package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.util.ElevatorMechanismVisualizer;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kG");

  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("Elevator/maxVelocityMetersPerSec", 2.0);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Elevator/maxAccelerationMetersPerSec", 10.0);

  static {
    switch (Constants.getRobotType()) {
      case ROBOT_2025_COMP -> {
        kP.initDefault(ElevatorConstants.Real.kP);
        kD.initDefault(ElevatorConstants.Real.kD);
        kS.initDefault(ElevatorConstants.Real.kS);
        kG.initDefault(ElevatorConstants.Real.kG);
        kV.initDefault(ElevatorConstants.Real.kV);
      }
      case ROBOT_SIMBOT -> {
        kP.initDefault(ElevatorConstants.Sim.kP);
        kD.initDefault(ElevatorConstants.Sim.kD);
        kS.initDefault(ElevatorConstants.Sim.kS);
        kG.initDefault(ElevatorConstants.Sim.kG);
        kV.initDefault(ElevatorConstants.Sim.kV);
      }
    }
  }

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final SysIdRoutine sysId;

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

  private Constraints elevatorConstraints =
      new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get());
  private ElevatorState setpoint = ElevatorState.STARTING_STATE;
  private ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0);
  private final ProfiledPIDController feedback =
      new ProfiledPIDController(0, 0, 0, elevatorConstraints);

  public Elevator(ElevatorIO elevatorIO) {
    this.io = elevatorIO;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)), null, this, "Elevator"));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    if (kS.hasChanged(0) || kG.hasChanged(0) || kV.hasChanged(0)) {
      feedforward = new ElevatorFeedforward(kS.get(), kG.get(), kV.get());
    }

    if (kP.hasChanged(0) || kD.hasChanged(0)) {
      feedback.setP(kP.get());
      feedback.setD(kD.get());
    }

    if (maxVelocity.hasChanged(0) || maxAcceleration.hasChanged(0)) {
      feedback.setConstraints(
          new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
    }

    if (DriverStation.isDisabled()) {
      setpoint = ElevatorState.STARTING_STATE;

      io.stop();

      feedback.reset(inputs.positionMeters, inputs.velocityRadPerSec);
    } else if (setpoint != null) {
      double measurement = inputs.positionMeters;
      double goal = setpoint.positionMeters();

      io.setVoltage(
          MathUtil.clamp(
              feedforward.calculate(0) + feedback.calculate(measurement, goal),
              -12,
              12)); // TODO idk what to do here but maybe this is right???
    }

    if (setpoint != null) {
      setpointVisualizer.update(setpoint.positionMeters());
    }
    measuredVisualizer.update(inputs.positionMeters);
  }

  public void setSetpoint(ElevatorState state) {
    setpoint = state;
  }

  @AutoLogOutput(key = "Elevator/Setpoint")
  public ElevatorState getSetpoint() {
    return setpoint;
  }

  @AutoLogOutput(key = "Elevator/CurrentState")
  public ElevatorState getCurrentState() {
    return new ElevatorState(inputs.positionMeters);
  }

  @AutoLogOutput(key = "Elevator/AtSetpoint")
  public boolean atSetpoint() {
    return getSetpoint().equals(getCurrentState());
  }

  public Command characterizeQuasistatic(SysIdRoutine.Direction direction) {
    return handleCharacterization().andThen(sysId.quasistatic(direction));
  }

  public Command characterizeDynamic(SysIdRoutine.Direction direction) {
    return handleCharacterization().andThen(sysId.dynamic(direction));
  }

  private Command handleCharacterization() {
    return Commands.runOnce(
        () -> {
          setpoint = null;
          io.stop();
        },
        this);
  }
}
