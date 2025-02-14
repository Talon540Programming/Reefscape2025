package frc.robot.subsystems.superstructure.elevator;

import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

class Elevator {
  // Tunable numbers
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA");

  // private static final LoggedTunableNumber maxVelocityMetersPerSec =
  //     new LoggedTunableNumber("Elevator/MaxVelocityMetersPerSec", 2.0);
  // private static final LoggedTunableNumber maxAccelerationMetersPerSec2 =
  //     new LoggedTunableNumber("Elevator/MaxAccelerationMetersPerSec2", 10);
  // private static final LoggedTunableNumber homingVolts =
  //     new LoggedTunableNumber("Elevator/HomingVolts", -2.0);
  // private static final LoggedTunableNumber homingTimeSecs =
  //     new LoggedTunableNumber("Elevator/HomingTimeSecs", 0.25);
  // private static final LoggedTunableNumber homingVelocityThresh =
  //     new LoggedTunableNumber("Elevator/HomingVelocityThresh", 5.0);
  // private static final LoggedTunableNumber staticCharacterizationVelocityThresh =
  //     new LoggedTunableNumber("Elevator/StaticCharacterizationVelocityThresh", 0.1);
  // private static final LoggedTunableNumber tolerance =
  //     new LoggedTunableNumber("Elevator/Tolerance", 0.2);

  static {
    switch (Constants.getRobot()) {
      case COMPBOT -> {
        kP.initDefault(0); // TODO
        kD.initDefault(0); // TODO
        kS.initDefault(0); // TODO
        kG.initDefault(0); // TODO
        kA.initDefault(0); // TODO
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

  // private final ElevatorIO io;
  // private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  @AutoLogOutput(key="Elevator/Homed")
  @Getter
  private boolean homed = false;





}
