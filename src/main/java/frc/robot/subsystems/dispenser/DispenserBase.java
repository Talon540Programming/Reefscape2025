package frc.robot.subsystems.dispenser;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class DispenserBase extends SubsystemBase {
  private static final LoggedTunableNumber coralIntakeVolts =
      new LoggedTunableNumber("Dispenser/CoralIntakeVolts", 6.0);
  private static final LoggedTunableNumber holdingCoralPeriod =
      new LoggedTunableNumber("Dispenser/HoldingCoralPeriodSecs", 0.5);

  private static final LoggedTunableNumber coralL1EjectVolts =
      new LoggedTunableNumber("Dispenser/CoralL1EjectVolts", 1.3);
  private static final LoggedTunableNumber coralEjectVolts =
      new LoggedTunableNumber("Dispenser/CoralEjectVolts", 3.0);
  private static final LoggedTunableNumber coralL4EjectVolts =
      new LoggedTunableNumber("Dispenser/CoralL4EjectVolts", 6.0); // TODO

  private static final LoggedTunableNumber ejectPeriod =
      new LoggedTunableNumber("Dispenser/CoralEjectPeriodSecs", 0.5);

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
    return startEnd(() -> io.runVolts(coralIntakeVolts.get()), io::stop)
        .raceWith(
            Commands.sequence(
                Commands.waitSeconds(0.25), Commands.waitUntil(this::isHoldingCoral)));
  }

  public Command eject(Supplier<ElevatorState> state) {
    return runRollers(
            switch (state.get()) {
              case L1_CORAL -> coralL1EjectVolts.get();
              case L4_CORAL -> coralL4EjectVolts.get();
              default -> coralEjectVolts.get();
            })
        .withTimeout(ejectPeriod.get());
  }
}
