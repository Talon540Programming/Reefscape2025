package frc.robot.subsystems.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean isConnected = false;
    public boolean hasResult = false;
    public double timestampSeconds = 0.0;
    public Pose3d estimatedRobotPose = new Pose3d();
    public Matrix<N3, N1> visionMeasurementStdDevs =
        MatBuilder.fill(Nat.N3(), Nat.N1(), 0.0, 0.0, 0.0);
    public int[] detectedTagsIds = new int[] {};
    public Pose3d[] detectedTagPoses = new Pose3d[] {};
    public double[] tagDistances = new double[] {};
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
