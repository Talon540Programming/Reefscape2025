package frc.robot.subsystems.dispenser;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.util.Debouncer;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class DispenserBase extends SubsystemBase {
  private static final LoggedTunableNumber intakeVolts =
      new LoggedTunableNumber("Dispenser/IntakeVolts", 6.0);
  private static final LoggedTunableNumber ejectVolts =
      new LoggedTunableNumber("Dispenser/EjectVolts", 1.3);
  private static final LoggedTunableNumber ejectVoltsSlow =
      new LoggedTunableNumber("Dispenser/EjectVoltsSlow", 1.3);

  private static final LoggedTunableNumber holdingCoralPeriod =
      new LoggedTunableNumber("Dispenser/HoldingCoralPeriodSecs", 0.5);

  private static final LoggedTunableNumber ejectPeriod =
      new LoggedTunableNumber("Dispenser/EjectPeriodSecs", 0.75);

  private final DispenserIO io;
  private final DispenserIOInputsAutoLogged inputs = new DispenserIOInputsAutoLogged();

  @Getter private boolean holdingCoral;

  private final Debouncer holdingCoralDebouncer = new Debouncer(0);

  private final Alert disconnected =
      new Alert("Dispenser motor disconnected!", Alert.AlertType.kWarning);

  public DispenserBase(DispenserIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Dispenser", inputs);

    disconnected.set(!inputs.connected);

    if (holdingCoralPeriod.hasChanged()) {
      holdingCoralDebouncer.setDebounceTime(holdingCoralPeriod.get());
    }

    // Update if holding coral.
    holdingCoral = holdingCoralDebouncer.calculate(inputs.rearBeamBreakBroken);
  }

  public Command runRollers(double inputVolts) {
    return startEnd(() -> io.runVolts(inputVolts), io::stop);
  }

  public Command intakeTillHolding() {
    return startEnd(() -> io.runVolts(intakeVolts.get()), io::stop).until(() -> holdingCoral);
  }

  public Command eject(Supplier<ElevatorState> state) {
    return startEnd(
            () ->
                io.runVolts(
                    state.get() == ElevatorState.L1_CORAL
                        ? ejectVoltsSlow.get()
                        : ejectVolts.get()),
            io::stop)
        .withTimeout(ejectPeriod.get());
  }
}
