package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeBase extends SubsystemBase {
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
  }

  public Command runRoller(double inputVolts) {
    return startEnd(() -> io.runVolts(inputVolts), io::stop);
  }
}
