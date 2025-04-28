package frc.robot.subsystems.dispenser;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DispenserBase extends SubsystemBase {
  public static final LoggedTunableNumber coralIntakeVolts =
      new LoggedTunableNumber("Dispenser/CoralIntakeVolts", 4.0);
  public static final LoggedTunableNumber coralIntakeWaitPeriod =
      new LoggedTunableNumber("Dispenser/CoralIntakeWaitPeriod", 0.5);
  public static final LoggedTunableNumber[] coralDispenseVolts = {
    new LoggedTunableNumber("Dispenser/TunnelDispenseVolts/L1", 1.75),
    new LoggedTunableNumber("Dispenser/TunnelDispenseVolts/L2", 2.5),
    new LoggedTunableNumber("Dispenser/TunnelDispenseVolts/L3", 2.5),
    new LoggedTunableNumber("Dispenser/TunnelDispenseVolts/L4", 6.0)
  };

  private final DispenserIO io;
  private final DispenserIOInputsAutoLogged inputs = new DispenserIOInputsAutoLogged();

  @AutoLogOutput private boolean holdingCoral;

  private static final double coralDebounceTime = 0.1;
  private final Debouncer holdingCoralDebouncer =
      new Debouncer(coralDebounceTime, DebounceType.kRising);

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

    // Update if holding coral.
    holdingCoral = holdingCoralDebouncer.calculate(inputs.rearBeamBreakBroken);

    // Record cycle time
    LoggedTracer.record("Dispenser");
  }

  public boolean holdingCoral() {
    return holdingCoral;
  }

  public Command runDispenser(double inputVolts) {
    return startEnd(() -> io.runVolts(inputVolts), io::stop);
  }

  public Command runDispenser(DoubleSupplier inputVolts) {
    return runEnd(() -> io.runVolts(inputVolts.getAsDouble()), io::stop);
  }

  public static double getScoringVoltage(ReefLevel level) {
    return coralDispenseVolts[level.ordinal()].get();
  }
}
