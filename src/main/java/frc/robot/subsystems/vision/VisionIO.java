package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import lombok.Builder;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean ntConnected = false;
    public PoseObservation[] observations = new PoseObservation[0];
    public int[][] detectedTagIds = new int[][] {};
  }

  @Builder
  public static record PoseObservation(
      double timestamp,
      boolean hasResult,
      Transform3d multitagTagToCamera,
      boolean isMultitag,
      Transform3d bestTagToCamera,
      Transform3d altTagToCamera,
      double ambiguity) {}

  public default void updateInputs(VisionIOInputs inputs) {}
}
