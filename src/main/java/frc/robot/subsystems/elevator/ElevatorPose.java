package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.subsystems.dispenser.DispenserConstants;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;

public record ElevatorPose(DoubleSupplier elevatorHeight) {
  private static final LoggedTunableNumber intakeHeightBaseline =
      new LoggedTunableNumber("Superstructure/Intake/ElevatorBaseline", 0.0);
  private static final LoggedTunableNumber intakeHeightRange =
      new LoggedTunableNumber("Elevator/Intake/ElevatorRange", Units.inchesToMeters(0.25)); // TODO
  private static final LoggedTunableNumber intakeHeightTimeFactor =
      new LoggedTunableNumber("Elevator/Intake/ElevatorTimeFactor", 25.0);
  private static final LoggedTunableNumber l1Height =
      new LoggedTunableNumber("Elevator/ReefScore/L1Height", 0.5); // TODO

  private static final Map<ReefLevel, LoggedTunableNumber> ejectDistance = new HashMap<>();
  private static final Map<ReefLevel, LoggedTunableNumber> heightFudges = new HashMap<>();

  private static void addInitialValue(
      Map<ReefLevel, LoggedTunableNumber> map,
      ReefLevel reefLevel,
      double initialValue,
      String key) {
    map.put(
        reefLevel,
        new LoggedTunableNumber("Elevator/ReefScore/" + key + "/" + reefLevel, initialValue));
  }

  static {
    // Coral eject distance
    addInitialValue(ejectDistance, ReefLevel.L2, 0.15, "EjectDistance"); // TODO
    addInitialValue(ejectDistance, ReefLevel.L3, 0.10, "EjectDistance"); // TODO
    addInitialValue(ejectDistance, ReefLevel.L4, 0.065, "EjectDistance"); // TODO

    // Height Fudge Factors
    addInitialValue(
        heightFudges,
        ReefLevel.L2,
        Units.inchesToMeters(2.5), // TODO
        "HeightFudges");
    addInitialValue(
        heightFudges,
        ReefLevel.L3,
        Units.inchesToMeters(2.5), // TODO
        "HeightFudges");
    addInitialValue(
        heightFudges,
        ReefLevel.L4,
        Units.inchesToMeters(1.0), // TODO
        "HeightFudges");
  }

  @RequiredArgsConstructor
  enum Preset {
    START(intakeHeightBaseline),
    STOW(intakeHeightBaseline),
    CORAL_INTAKE(
        () ->
            intakeHeightBaseline.get()
                + intakeHeightRange.get()
                    * Math.sin(Timer.getTimestamp() * intakeHeightTimeFactor.get())),
    STUCK_CORAL("StuckCoral", 0.2),
    L1_CORAL(l1Height),
    L2_CORAL(ReefLevel.L2),
    L3_CORAL(ReefLevel.L3),
    L4_CORAL(ReefLevel.L4);

    private final ElevatorPose pose;

    Preset(DoubleSupplier elevatorHeight) {
      this(new ElevatorPose(elevatorHeight));
    }

    Preset(String name, double elevatorHeight) {
      this(new LoggedTunableNumber("Elevator/" + name + "/Height", elevatorHeight));
    }

    Preset(ReefLevel reefLevel) {
      // // new Pose2d(0.0, reefLevel.height + (Constants.getRobot() == Constants.RobotType.SIMBOT ? 0.0 : heightFudges.get(reefLevel).get()),
      //     DispenserConstants.dispenserAngle
      // );

      // TODO
      this(new ElevatorPose(() -> 0));
    }

    public double getElevatorHeight() {
      return pose.elevatorHeight.getAsDouble();
    }
  }
}
