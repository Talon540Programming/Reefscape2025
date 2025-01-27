package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLog;

public class OdometryManager implements AutoCloseable {
  public static Lock odometryLock =
      new ReentrantLock(); // Prevent conflicts when reading and writing data

  private static OdometryManager instance = null;

  public static OdometryManager getInstance() {
    if (instance == null) {
      instance = new OdometryManager();
    }
    return instance;
  }

  @Getter private final Queue<Double> timestampQueue = new ArrayBlockingQueue<>(20);
  private final List<DoubleSupplier> signalSuppliers = new ArrayList<>(9);
  private final List<Queue<Double>> signalQueues = new ArrayList<>(9);

  private final Notifier notifier = new Notifier(this::run);

  private OdometryManager() {
    notifier.setName("Odometry Data Collection Thread");
  }

  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    odometryLock.lock();
    try {
      signalSuppliers.add(signal);
      signalQueues.add(queue);
    } finally {
      odometryLock.unlock();
    }
    return queue;
  }

  private void run() {
    odometryLock.lock();
    try {
      // Get sample timestamp
      double timestamp = RobotController.getFPGATime() / 1e6;
      timestampQueue.offer(timestamp);

      // Read signals and provide them to queues
      for (int i = 0; i < signalSuppliers.size(); i++) {
        signalQueues.get(i).offer(signalSuppliers.get(i).getAsDouble());
      }
    } finally {
      odometryLock.unlock();
    }
  }

  public void start() throws IllegalStateException {
    // Check that any sources are supplied
    if (signalSuppliers.isEmpty()) {
      throw new IllegalStateException(
          "Tried to start OdometryManager, but no sources have been set.");
    }

    notifier.startPeriodic(1.0 / DriveConstants.odometryFrequencyHz);
  }

  @Override
  public void close() {
    notifier.stop();
    notifier.close();
  }

  @AutoLog
  public static class OdometryTimestampsInput {
    public double[] timestamps;
  }
}
