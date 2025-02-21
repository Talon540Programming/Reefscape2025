package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.Alert;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

// Open loop until that rear beam break is broken, then we know the game piece is sticking out x
// inches. then reset position of encoder and use PID to control corals position within the end
// effector

class Effector {
  public static final LoggedTunableNumber intakeVolts =
      new LoggedTunableNumber("Effector/IntakeVolts", 3.0);
  public static final LoggedTunableNumber ejectVolts =
      new LoggedTunableNumber("Effector/EjectVolts", 6.0);

  private final EffectorIO io;
  private final EffectorIOInputsAutoLogged inputs = new EffectorIOInputsAutoLogged();

  private final Alert disconnected =
      new Alert("Effector motor disconnected!", Alert.AlertType.kWarning);

  public Effector(EffectorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Effector", inputs);

    disconnected.set(!inputs.connected);
  }

  public void runVolts(double output) {
    io.runVolts(output);
  }
}

