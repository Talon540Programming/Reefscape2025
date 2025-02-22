package frc.robot.subsystems.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface VisionIO {
  public static class VisionIOInputs implements LoggableInputs, Cloneable {
    public boolean isConnected = false;
    public boolean hasResult = false;
    public double timestampSeconds = 0.0;
    public Pose3d estimatedRobotPose = new Pose3d();
    public Matrix<N3, N1> visionMeasurementStdDevs =
        MatBuilder.fill(Nat.N3(), Nat.N1(), 0.0, 0.0, 0.0);
    public int[] detectedTagsIds = new int[] {};
    public Pose3d[] detectedTagPoses = new Pose3d[] {};

    @Override
    public void toLog(LogTable table) {
      table.put("IsConnected", isConnected);
      table.put("HasResult", hasResult);
      table.put("TimestampSeconds", timestampSeconds);
      table.put("EstimatedRobotPose", estimatedRobotPose);
      table.put("DetectedTagsIds", detectedTagsIds);
      table.put("DetectedTagPoses", detectedTagPoses);
      table.put("VisionMeasurementStdDevs", visionMeasurementStdDevs.getData());
    }

    @Override
    public void fromLog(LogTable table) {
      isConnected = table.get("IsConnected", isConnected);
      hasResult = table.get("HasResult", hasResult);
      timestampSeconds = table.get("TimestampSeconds", timestampSeconds);
      estimatedRobotPose = table.get("EstimatedRobotPose", estimatedRobotPose);
      detectedTagsIds = table.get("DetectedTagsIds", detectedTagsIds);
      detectedTagPoses = table.get("DetectedTagPoses", detectedTagPoses);
      visionMeasurementStdDevs =
          MatBuilder.fill(
              Nat.N3(),
              Nat.N1(),
              table.get("VisionMeasurementStdDevs", visionMeasurementStdDevs.getData()));
    }

    public VisionIOInputs clone() {
      VisionIOInputs copy = new VisionIOInputs();
      copy.isConnected = this.isConnected;
      copy.hasResult = this.hasResult;
      copy.timestampSeconds = this.timestampSeconds;
      copy.estimatedRobotPose = this.estimatedRobotPose;
      copy.detectedTagsIds = this.detectedTagsIds.clone();
      copy.detectedTagPoses = this.detectedTagPoses.clone();
      copy.visionMeasurementStdDevs = this.visionMeasurementStdDevs.copy();
      return copy;
    }
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
