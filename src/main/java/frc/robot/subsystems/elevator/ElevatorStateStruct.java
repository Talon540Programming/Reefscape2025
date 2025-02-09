package frc.robot.subsystems.elevator;

import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class ElevatorStateStruct implements Struct<ElevatorState> {
  @Override
  public Class<ElevatorState> getTypeClass() {
    return ElevatorState.class;
  }

  @Override
  public String getTypeName() {
    return "ElevatorState";
  }

  @Override
  public int getSize() {
    return kSizeDouble;
  }

  @Override
  public String getSchema() {
    return "double positionMeters";
  }

  @Override
  public ElevatorState unpack(ByteBuffer byteBuffer) {
    var positionMeters = byteBuffer.getDouble();
    return new ElevatorState(positionMeters);
  }

  @Override
  public void pack(ByteBuffer byteBuffer, ElevatorState elevatorState) {
    byteBuffer.putDouble(elevatorState.positionMeters());
  }
}
