package frc.robot.subsystems.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface VisionIO {
  public static class VisionIOInputs implements LoggableInputs, Cloneable {
    // public int pipelineIndex =
    //     0; // 0 is Apriltags (3d detection), 1 is corner detection (2d detection)
    public boolean isConnected = false;
    public boolean hasResult = false;
    public double timestampSeconds = 0.0;
    public Matrix<N3, N1> visionMeasurementStdDevs =
        MatBuilder.fill(Nat.N3(), Nat.N1(), 0.0, 0.0, 0.0);
    public int[] detectedTagsIds = new int[] {};
    public Translation2d[] detectedCorners = new Translation2d[] {};
    public double tagHorizontalDistance = 0.0;
    public double tagCenterX = 0.0;

    @Override
    public void toLog(LogTable table) {
      table.put("IsConnected", isConnected);
      table.put("TimestampSeconds", timestampSeconds);
      table.put("DetectedTagsIds", detectedTagsIds);
      // table.put("AprilTag/HasResult", hasAprilTagResult);
      // table.put("AprilTag/EstimatedRobotPose", estimatedRobotPose);
      // table.put("AprilTag/DetectedTagPoses", detectedTagPoses);
      // table.put("AprilTag/VisionMeasurementStdDevs", visionMeasurementStdDevs.getData());
      table.put("HasResult", hasResult);
      table.put("DetectedCorners", detectedCorners);
      table.put("TagDistance", tagHorizontalDistance);
      table.put("Corners/tagCenterX", tagCenterX);
      // table.put("Pipeline", pipelineIndex);
    }

    @Override
    public void fromLog(LogTable table) {
      isConnected = table.get("IsConnected", isConnected);
      timestampSeconds = table.get("TimestampSeconds", timestampSeconds);
      detectedTagsIds = table.get("DetectedTagsIds", detectedTagsIds);
      visionMeasurementStdDevs =
          MatBuilder.fill(
              Nat.N3(),
              Nat.N1(),
              table.get("VisionMeasurementStdDevs", visionMeasurementStdDevs.getData()));
      hasResult = table.get("Corners/HasResult", hasResult);
      detectedCorners = table.get("Corners/DetectedCorners", detectedCorners);
      tagHorizontalDistance = table.get("Corners/TagDistance", tagHorizontalDistance);
      tagCenterX = table.get("Corners/tagCenterX", tagCenterX);
      // pipelineIndex = table.get("Pipeline", pipelineIndex);
    }

    public VisionIOInputs clone() {
      VisionIOInputs copy = new VisionIOInputs();
      copy.isConnected = this.isConnected;
      copy.hasResult = this.hasResult;
      copy.timestampSeconds = this.timestampSeconds;
      copy.detectedTagsIds = this.detectedTagsIds.clone();
      copy.visionMeasurementStdDevs = this.visionMeasurementStdDevs.copy();
      copy.tagHorizontalDistance = this.tagHorizontalDistance;
      copy.detectedCorners = this.detectedCorners.clone();
      copy.tagCenterX = this.tagCenterX;
      // copy.pipelineIndex = this.pipelineIndex;
      return copy;
    }
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default void updateCorners(VisionIOInputs inputs) {}

  // public default void updateAprilTag(VisionIOInputs inputs) {}

  // public default void setPipelineIndex(int index) {}

  public default Transform3d getRobotToCamera() {
    return new Transform3d();
  }
}
