package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

public class OdometryManager {
  public static Lock odometryLock = new ReentrantLock();
  public static double ODOMETRY_FREQUENCY_HZ = 200.0;

  private static OdometryManager instance = null;

  public static OdometryManager getInstance() {
    if (instance == null) {
      instance = new OdometryManager();
    }
    return instance;
  }

  private final Queue<Double> timestampQueue = new ArrayBlockingQueue<>(20);
  private final List<ModuleSource> m_moduleSources = new ArrayList<>(4);
  private GyroSource m_gyroSource = null;

  public void registerModuleSource(
      Supplier<Optional<Double>> drivePositionSupplier,
      Supplier<Optional<Rotation2d>> turnAngleSupplier) {}

  public void registerGyroSource(Supplier<Optional<Rotation2d>> robotYawSupplier) {}

  private static void run() {}

  private record ModuleSource(
      Queue<Double> drivePositionQueue,
      Supplier<Optional<Double>> drivePositionSupplier,
      Queue<Rotation2d> turnAngleQueue,
      Supplier<Optional<Rotation2d>> turnAngleSupplier) {}

  private record GyroSource(
      Queue<Rotation2d> robotYawQueue, Supplier<Optional<Rotation2d>> robotYawSupplier) {}

  @AutoLog
  public class TimestampInputs {
    public double[] timestamps;

    public Rotation2d[] gyroYaws;
    public SwerveModulePosition[][] modulePositions;
  }
}
