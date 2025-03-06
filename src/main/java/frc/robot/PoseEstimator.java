package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.drive.DriveBase;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class PoseEstimator {
  // Standard deviations of the pose estimate (x position in meters, y position in meters, and
  // heading in radians).
  // Increase these numbers to trust your state estimate less.
  private static final Matrix<N3, N1> odometryStateStdDevs = VecBuilder.fill(0.003, 0.003, 0.002);
  private static final double poseBufferSizeSec = 2.0;

  private static PoseEstimator instance;

  public static PoseEstimator getInstance() {
    if (instance == null) {
      instance = new PoseEstimator();
    }
    return instance;
  }

  @Getter
  @AutoLogOutput(key = "PoseEstimator/OdometryPose")
  private Pose2d odometryPose = new Pose2d();

  @Getter
  @AutoLogOutput(key = "PoseEstimator/EstimatedPose")
  private Pose2d estimatedPose = new Pose2d();

  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);
  private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());

  // Odometry
  private final SwerveDriveKinematics kinematics;
  private SwerveModulePosition[] lastWheelPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  // Assume gyro starts at zero
  private Rotation2d gyroOffset = new Rotation2d();

  private PoseEstimator() {
    for (int i = 0; i < 3; ++i) {
      qStdDevs.set(i, 0, Math.pow(odometryStateStdDevs.get(i, 0), 2));
    }

    kinematics = new SwerveDriveKinematics(DriveBase.getModuleTranslations());
  }

  public void resetPose(Pose2d pose) {
    // Gyro offset is the rotation that maps the old gyro rotation (estimated - offset) to the new
    // frame of rotation
    gyroOffset = pose.getRotation().minus(estimatedPose.getRotation().minus(gyroOffset));
    estimatedPose = pose;
    odometryPose = pose;
    poseBuffer.clear();
  }

  public void addOdometryObservation(
      SwerveModulePosition[] wheelPositions, Rotation2d gyroAngle, double timestamp) {
    var twist = kinematics.toTwist2d(lastWheelPositions, wheelPositions);

    // Update previous state
    lastWheelPositions = wheelPositions;
    Pose2d lastOdometryPose = odometryPose;

    // Update Odometry
    odometryPose = odometryPose.exp(twist);

    if (gyroAngle != null) {
      // Use gyro measurement
      // Add offset to measured angle
      Rotation2d angle = gyroAngle.plus(gyroOffset);
      odometryPose = new Pose2d(odometryPose.getTranslation(), angle);
    }

    // Add pose to buffer at timestamp
    poseBuffer.addSample(timestamp, odometryPose);

    // Calculate diff from last odometry pose and add onto pose estimate
    Twist2d finalTwist = lastOdometryPose.log(odometryPose);
    estimatedPose = estimatedPose.exp(finalTwist);
  }

  public Rotation2d getRotation() {
    return estimatedPose.getRotation();
  }
}
