package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@Getter
@RequiredArgsConstructor
public enum ElevatorState {
  START(() -> 0.0),
  STOW("Stow", 0.0),
  INTAKE("Intake", 0.0),
  L1_CORAL(ReefLevel.L1, Units.inchesToMeters(7), false),
  L2_CORAL(ReefLevel.L2, Units.inchesToMeters(3), false),
  L2_ALGAE_REMOVAL(ReefLevel.L2, Units.inchesToMeters(5), true),
  L3_CORAL(ReefLevel.L3, Units.inchesToMeters(2.0), false),
  L3_ALGAE_REMOVAL(ReefLevel.L3, Units.inchesToMeters(0), true),
  L4_CORAL(ReefLevel.L4, Units.inchesToMeters(0.0), false);

  private final DoubleSupplier elevatorHeightMeters;

  ElevatorState(String name, double defaultValue) {
    elevatorHeightMeters = new LoggedTunableNumber("Elevator/Presets/" + name, defaultValue);
  }

  ElevatorState(ReefLevel reefLevel, double defaultOffset, boolean isAlgae) {
    String stateName;
    if (isAlgae) {
      stateName = String.format("Elevator/Presets/%s_Algae Offset", reefLevel);
    } else {
      stateName = String.format("Elevator/Presets/%s Offset", reefLevel);
    }

    var offsetTunable = new LoggedTunableNumber(stateName, defaultOffset);
    elevatorHeightMeters =
        () ->
            (reefLevel.height + offsetTunable.get() - originToBaseHeightMeters)
                / elevatorPitch.getSin();
  }
}
