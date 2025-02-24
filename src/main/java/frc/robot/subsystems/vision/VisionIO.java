package frc.robot.subsystems.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface VisionIO {
  public static class VisionIOInputs implements LoggableInputs, Cloneable {
    public int pipelineIndex = 0; // 0 is Apriltags (3d detection), 1 is corner detection (2d detection)
    public boolean isConnected = false;
    public boolean hasAprilTagResult = false;
    public boolean hasCornersResult = false;
    public double timestampSeconds = 0.0;
    public Pose3d estimatedRobotPose = new Pose3d();
    public Matrix<N3, N1> visionMeasurementStdDevs =
        MatBuilder.fill(Nat.N3(), Nat.N1(), 0.0, 0.0, 0.0);
    public int[] detectedTagsIds = new int[] {};
    public Pose3d[] detectedTagPoses = new Pose3d[] {};
    public Translation2d[] detectedCorners = new Translation2d[] {};
    public double tagDistance = 0.0;


    @Override
    public void toLog(LogTable table) {
      table.put("IsConnected", isConnected);
      table.put("TimestampSeconds", timestampSeconds);
      table.put("DetectedTagsIds", detectedTagsIds);
      table.put("AprilTag/HasResult", hasAprilTagResult);
      table.put("AprilTag/EstimatedRobotPose", estimatedRobotPose);
      table.put("AprilTag/DetectedTagPoses", detectedTagPoses);
      table.put("AprilTag/VisionMeasurementStdDevs", visionMeasurementStdDevs.getData());
      table.put("Corners/HasResult", hasCornersResult);
      table.put("Corners/DetectedCorners", detectedCorners);
      table.put("Corners/TagDistance", tagDistance);
      table.put("Pipeline", pipelineIndex);
    }

    @Override
    public void fromLog(LogTable table) {
      isConnected = table.get("IsConnected", isConnected);
      timestampSeconds = table.get("TimestampSeconds", timestampSeconds);
      detectedTagsIds = table.get("DetectedTagsIds", detectedTagsIds);
      hasAprilTagResult = table.get("AprilTag/HasResult", hasAprilTagResult);
      estimatedRobotPose = table.get("AprilTag/EstimatedRobotPose", estimatedRobotPose);
      detectedTagPoses = table.get("AprilTag/DetectedTagPoses", detectedTagPoses);
      visionMeasurementStdDevs =
          MatBuilder.fill(
              Nat.N3(),
              Nat.N1(),
              table.get("VisionMeasurementStdDevs", visionMeasurementStdDevs.getData()));
      hasCornersResult = table.get("Corners/HasResult", hasCornersResult);
      detectedCorners = table.get("Corners/DetectedCorners", detectedCorners);
      tagDistance = table.get("Corners/TagDistance", tagDistance);
      pipelineIndex = table.get("Pipeline", pipelineIndex);
    }

    public VisionIOInputs clone() {
      VisionIOInputs copy = new VisionIOInputs();
      copy.isConnected = this.isConnected;
      copy.hasAprilTagResult = this.hasAprilTagResult;
      copy.hasCornersResult = this.hasCornersResult;
      copy.timestampSeconds = this.timestampSeconds;
      copy.estimatedRobotPose = this.estimatedRobotPose;
      copy.detectedTagsIds = this.detectedTagsIds.clone();
      copy.detectedTagPoses = this.detectedTagPoses.clone();
      copy.visionMeasurementStdDevs = this.visionMeasurementStdDevs.copy();
      copy.tagDistance = this.tagDistance;
      copy.detectedCorners = this.detectedCorners.clone();
      copy.pipelineIndex = this.pipelineIndex;
      return copy;
    }
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default void updateCorners(VisionIOInputs inputs) {}

  public default void updateAprilTag(VisionIOInputs inputs) {}

  public default void setPipelineIndex(int index) {}

  public default Transform3d getRobotToCamera() { return new Transform3d(); }
} 
