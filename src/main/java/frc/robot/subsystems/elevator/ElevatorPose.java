package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import java.util.function.Supplier;
import lombok.RequiredArgsConstructor;

public record ElevatorPose(DoubleSupplier elevatorHeight) {
  private static final LoggedTunableNumber intakeHeightBaseline =
      new LoggedTunableNumber("Elevator/Intake/ElevatorBaseline", 0.0);
  private static final LoggedTunableNumber intakeHeightAmplitude =
      new LoggedTunableNumber("Elevator/Intake/ElevatorAmplitude", 0.0);
  private static final LoggedTunableNumber intakeHeightPeriod =
      new LoggedTunableNumber("Elevator/Intake/ElevatorPeriod", 0.25);
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
    // Coral eject distance. Currently, assumes flush to reef.
    addInitialValue(ejectDistance, ReefLevel.L2, 0.21, "EjectDistance");
    addInitialValue(ejectDistance, ReefLevel.L3, 0.185, "EjectDistance");
    addInitialValue(ejectDistance, ReefLevel.L4, 0.135, "EjectDistance");

    // Height Fudge Factors
    addInitialValue(heightFudges, ReefLevel.L2, -0.0275, "HeightFudges");
    addInitialValue(heightFudges, ReefLevel.L3, -0.05, "HeightFudges");
    addInitialValue(heightFudges, ReefLevel.L4, Units.inchesToMeters(1.0), "HeightFudges");
  }

  @RequiredArgsConstructor
  public enum CoralDispenserPose {
    L2_CORAL(ReefLevel.L2),
    L3_CORAL(ReefLevel.L3),
    L4_CORAL(ReefLevel.L4);

    private final Supplier<Pose2d> pose;

    CoralDispenserPose(ReefLevel reefLevel) {
      pose = () -> calculatePose(reefLevel);
    }

    public double getElevatorHeight() {
      return (pose.get().getY() - ElevatorConstants.dispenserOrigin2d.getY())
          / ElevatorConstants.elevatorAngle.getSin();
    }

    public Transform2d toRobotPose() {
      return new Transform2d(
          getElevatorHeight() * ElevatorConstants.elevatorAngle.getCos()
              + pose.get().getX()
              + ElevatorConstants.dispenserOrigin2d.getX(),
          0.0,
          Rotation2d.kPi);
    }

    // Calculate where the dispenser should be per the tunable parameters
    private static Pose2d calculatePose(ReefLevel reefLevel) {
      return new Pose2d(
          new Pose2d(
                  0.0,
                  reefLevel.height
                      + (Constants.getRobot() == Constants.RobotType.SIMBOT
                          ? 0.0
                          : heightFudges.get(reefLevel).get()),
                  DispenserConstants.dispenserAngle.unaryMinus())
              .transformBy(
                  GeomUtil.toTransform2d(
                      ejectDistance.get(reefLevel).get()
                          + DispenserConstants.elevatorToDispenserFront,
                      0.0))
              .getTranslation(),
          DispenserConstants.dispenserAngle);
    }
  }

  @RequiredArgsConstructor
  public enum Preset {
    START(intakeHeightBaseline),
    STOW(intakeHeightBaseline),
    CORAL_INTAKE(
        () ->
            intakeHeightBaseline.get()
                + intakeHeightAmplitude.get()
                    * Math.sin((2.0 * Math.PI) / intakeHeightPeriod.get() * Timer.getTimestamp())),
    STUCK_CORAL("StuckCoral", 0.15),
    L1_CORAL(l1Height),
    L2_CORAL(CoralDispenserPose.L2_CORAL),
    L3_CORAL(CoralDispenserPose.L3_CORAL),
    L4_CORAL(CoralDispenserPose.L4_CORAL);

    private final ElevatorPose pose;

    Preset(DoubleSupplier elevatorHeight) {
      this(new ElevatorPose(elevatorHeight));
    }

    Preset(String name, double elevatorHeight) {
      this(new LoggedTunableNumber("Elevator/" + name + "/Height", elevatorHeight));
    }

    Preset(CoralDispenserPose coralDispenserPose) {
      pose = new ElevatorPose(coralDispenserPose::getElevatorHeight);
    }

    public double getElevatorHeight() {
      return pose.elevatorHeight.getAsDouble();
    }
  }
}
