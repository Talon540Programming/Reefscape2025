package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.FieldConstants.AprilTagLayoutType;
import java.util.List;
import java.util.Optional;
import lombok.Builder;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.utils.PacketUtils;

public interface VisionIO {
  public static class VisionIOInputs implements LoggableInputs {
    public boolean ntConnected = false;
    public PoseObservation[] observations = new PoseObservation[0];

    @Override
    public void toLog(LogTable table) {
      table.put("NetworkTablesConnected", ntConnected);
      var writeBuffer = new byte[observations.length][];
      for (int i = 0; i < observations.length; i++) {
        var packet = new Packet(1);
        var observation = observations[i];
        packet.encode(observation.hasResult);
        packet.encode(observation.timestampSeconds);
        observation.multitagResult.ifPresentOrElse(
            multitagPose -> {
              packet.encode(true);
              PacketUtils.packTransform3d(packet, multitagPose.multitagTagToCamera);
            },
            () -> packet.encode(false));
        observation.singleTagResult.ifPresentOrElse(
            singletagPose -> {
              packet.encode(true);
              packet.encode(singletagPose.tagId);
              PacketUtils.packTransform3d(packet, singletagPose.bestTagToCamera);
              PacketUtils.packTransform3d(packet, singletagPose.altTagToCamera);
              packet.encode(singletagPose.ambiguity);
              PacketUtils.packRotation2d(packet, singletagPose.pitch);
              PacketUtils.packRotation2d(packet, singletagPose.yaw);
            },
            () -> packet.encode(false));
        packet.encode(observation.detectedTagIds);
        writeBuffer[i] = packet.getWrittenDataCopy();
      }
      table.put("data", writeBuffer);
    }

    @Override
    public void fromLog(LogTable table) {
      ntConnected = table.get("NetworkTablesConnected", ntConnected);
      var dataBuffer = table.get("data", new byte[0][]);
      observations = new PoseObservation[dataBuffer.length];
      if (observations.length == 0) return;

      for (int i = 0; i < dataBuffer.length; i++) {
        var packet = new Packet(1);
        var poseObservationBuilder = PoseObservation.builder();
        poseObservationBuilder
            .hasResult(packet.decodeBoolean())
            .timestampSeconds(packet.decodeDouble())
            .multitagResult(
                packet.decodeBoolean()
                    ? Optional.of(
                        new MultitagPoseObservation(PacketUtils.unpackTransform3d(packet)))
                    : Optional.empty())
            .singleTagResult(
                packet.decodeBoolean()
                    ? Optional.of(
                        new SingleTagPoseObservation(
                            packet.decodeInt(),
                            PacketUtils.unpackTransform3d(packet),
                            PacketUtils.unpackTransform3d(packet),
                            packet.decodeDouble(),
                            PacketUtils.unpackRotation2d(packet),
                            PacketUtils.unpackRotation2d(packet)))
                    : Optional.empty())
            .detectedTagIds(packet.decodeShortList());
        observations[i] = poseObservationBuilder.build();
      }
    }
  }

  @Builder
  public static class PoseObservation {
    public final boolean hasResult;
    public final double timestampSeconds;
    @Builder.Default public Optional<MultitagPoseObservation> multitagResult = Optional.empty();
    @Builder.Default public Optional<SingleTagPoseObservation> singleTagResult = Optional.empty();
    @Builder.Default public final List<Short> detectedTagIds = List.of();
  }

  public record MultitagPoseObservation(Transform3d multitagTagToCamera) {}

  public record SingleTagPoseObservation(
      int tagId,
      Transform3d bestTagToCamera,
      Transform3d altTagToCamera,
      double ambiguity,
      Rotation2d pitch,
      Rotation2d yaw) {}

  public default void updateInputs(VisionIOInputs inputs) {}

  public default void setAprilTagFieldLayout(AprilTagLayoutType layoutType) {}
}
