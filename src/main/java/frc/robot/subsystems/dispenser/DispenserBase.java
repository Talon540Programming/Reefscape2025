package frc.robot.subsystems.dispenser;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Debouncer;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class DispenserBase extends SubsystemBase {
  private static final LoggedTunableNumber intakeVolts =
      new LoggedTunableNumber("Dispenser/IntakeVolts", 6.0);
  private static final LoggedTunableNumber ejectVolts =
      new LoggedTunableNumber("Dispenser/EjectVolts", 1.3);

  private static final LoggedTunableNumber holdingCoralPeriod =
      new LoggedTunableNumber("Dispenser/HoldingCoralPeriodSecs", 0.5);

  private static final LoggedTunableNumber ejectPeriod =
      new LoggedTunableNumber("Dispenser/EjectPeriodSecs", 0.65);

  private final DispenserIO io;
  private final DispenserIOInputsAutoLogged inputs = new DispenserIOInputsAutoLogged();

  @Setter private boolean isEStopped;
  @Getter private boolean hasCoral;

  private final Debouncer holdingCoralDebouncer = new Debouncer(0);

  private final Alert disconnected =
      new Alert("Dispenser motor disconnected!", Alert.AlertType.kWarning);

  // @Setter private double coralVolts = 0.0;

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
    hasCoral = holdingCoralDebouncer.calculate(inputs.rearBeamBreakBroken);

    // // Run Coral
    // if (!isEStopped) {
    //   io.runVolts(coralVolts);
    // } else {
    //   io.stop();
    // }
  }

  public Command intakeTillHolding() {
    return startEnd(() -> io.runVolts(intakeVolts.get()), io::stop).until(() -> hasCoral);
  }

  public Command eject() {
    return startEnd(() -> io.runVolts(ejectVolts.get()), io::stop).withTimeout(ejectPeriod.get());
  }
}
