package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import lombok.Builder;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean ntConnected = false;

    public PoseObservation[] observations = new PoseObservation[0];
  }

  @Builder
  public static record PoseObservation(
      double timestamp,
      Transform3d multitagTagToCamera,
      Transform3d bestTagToCamera,
      Transform3d altTagToCamera,
      double ambiguity,
      int[] detectedTagsIds) {}

  public default void updateInputs(VisionIOInputs inputs) {}
}
