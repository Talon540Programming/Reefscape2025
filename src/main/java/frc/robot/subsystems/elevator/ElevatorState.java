package frc.robot.subsystems.elevator;

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
  L1_CORAL(ReefLevel.L1, Units.inchesToMeters(0.0)),
  L2_CORAL(ReefLevel.L2, Units.inchesToMeters(0.0)),
  L3_CORAL(ReefLevel.L3, Units.inchesToMeters(0.0));

  private final DoubleSupplier elevatorHeightMeters;

  ElevatorState(String name, double defaultValue) {
    elevatorHeightMeters = new LoggedTunableNumber("Elevator/Presets/" + name, defaultValue);
  }

  ElevatorState(ReefLevel reefLevel, double defaultOffset) {
    var offsetTunable =
        new LoggedTunableNumber(
            String.format("Elevator/Presets/%s Offset", reefLevel), defaultOffset);
    elevatorHeightMeters = () -> reefLevel.height + offsetTunable.get();
  }
}
