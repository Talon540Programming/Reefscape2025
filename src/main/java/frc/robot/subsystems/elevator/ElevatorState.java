package frc.robot.subsystems.elevator;

import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.constants.Constants;
import frc.robot.util.LoggedTunableNumber;

public record ElevatorState(double positionMeters) implements StructSerializable {
  public static final ElevatorState STARTING_STATE = new ElevatorState(0);
  public static final ElevatorState CORAL_INTAKE_STATE = new ElevatorState(0); // TODO
  public static final ElevatorState L1_STATE = new ElevatorState(0); // TODO
  public static final ElevatorState L2_STATE = new ElevatorState(0); // TODO
  public static final ElevatorState L3_STATE = new ElevatorState(0); // TODO

  private static final LoggedTunableNumber positionTolerance =
      new LoggedTunableNumber("Elevator/PositionTolerance");

  static {
    switch (Constants.getRobotType()) {
      case ROBOT_SIMBOT -> {
        positionTolerance.initDefault(0.01);
      }
      case ROBOT_2025_COMP -> {
        positionTolerance.initDefault(0.02);
      }
    }
  }

  @Override
  public boolean equals(Object obj) {
    if (obj instanceof ElevatorState other) {
      return Math.abs(positionMeters - other.positionMeters) <= positionTolerance.get();
    }
    return false;
  }

  public static final ElevatorStateStruct struct = new ElevatorStateStruct();
}
