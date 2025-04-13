package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class IntakeBase extends SubsystemBase {
  public static final LoggedTunableNumber intakeVolts =
      new LoggedTunableNumber("Intake/HopperIntakeVolts", 5.5);

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final Alert disconnected =
      new Alert("Intake motor disconnected!", Alert.AlertType.kWarning);

  public IntakeBase(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    disconnected.set(!inputs.connected);

    // Record cycle time
    LoggedTracer.record("Intake");
  }

  public Command runRoller(DoubleSupplier inputVolts) {
    return runEnd(() -> io.runVolts(inputVolts.getAsDouble()), io::stop);
  }

  public Command runRoller(double inputVolts) {
    return startEnd(() -> io.runVolts(inputVolts), io::stop);
  }
}
