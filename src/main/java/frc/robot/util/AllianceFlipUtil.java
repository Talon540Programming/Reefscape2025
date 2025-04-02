package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;
import java.util.function.Function;

public class AllianceFlipUtil {
  private static double applyX(double x) {
    return shouldFlip() ? FieldConstants.fieldLength - x : x;
  }

  public static double applyY(double y) {
    return shouldFlip() ? FieldConstants.fieldWidth - y : y;
  }

  private static Translation2d applyTranslation(Translation2d translation2d) {
    return new Translation2d(applyX(translation2d.getX()), applyY(translation2d.getY()));
  }

  private static Translation3d applyTranslation(Translation3d translation3d) {
    return new Translation3d(
        applyX(translation3d.getX()), applyY(translation3d.getY()), translation3d.getZ());
  }

  private static Rotation2d applyRotation(Rotation2d rotation2d) {
    return rotation2d.rotateBy(Rotation2d.kPi);
  }

  private static Rotation3d applyRotation(Rotation3d rotation3d) {
    return rotation3d.rotateBy(new Rotation3d(0.0, 0.0, Math.PI));
  }

  private static Pose2d applyPose(Pose2d pose) {
    return new Pose2d(applyTranslation(pose.getTranslation()), applyRotation(pose.getRotation()));
  }

  private static Pose3d applyPose(Pose3d pose3d) {
    return new Pose3d(
        applyTranslation(pose3d.getTranslation()), applyRotation(pose3d.getRotation()));
  }

  public static boolean shouldFlip() {
    var currentAllianceOpt = DriverStation.getAlliance();
    return currentAllianceOpt.isPresent() && currentAllianceOpt.get() == DriverStation.Alliance.Red;
  }

  public static double apply(double x) {
    return shouldFlip() ? applyX(x) : x;
  }

  public static Translation2d apply(Translation2d translation2d) {
    return shouldFlip() ? applyTranslation(translation2d) : translation2d;
  }

  public static Translation3d apply(Translation3d translation3d) {
    return shouldFlip() ? applyTranslation(translation3d) : translation3d;
  }

  public static Rotation2d apply(Rotation2d rotation2d) {
    return shouldFlip() ? applyRotation(rotation2d) : rotation2d;
  }

  public static Rotation3d apply(Rotation3d rotation3d) {
    return shouldFlip() ? applyRotation(rotation3d) : rotation3d;
  }

  public static Pose2d apply(Pose2d pose2d) {
    return shouldFlip() ? applyPose(pose2d) : pose2d;
  }

  public static Pose3d apply(Pose3d pose3d) {
    return shouldFlip() ? applyPose(pose3d) : pose3d;
  }

  public static class AllianceRelative<T> {
    private final T value;
    private final Function<T, T> transformer;

    public AllianceRelative(T value, Function<T, T> transformer) {
      this.value = value;
      this.transformer = transformer;
    }

    public T get() {
      return shouldFlip() ? transformer.apply(value) : value;
    }

    public T getRaw() {
      return value;
    }

    public static AllianceRelative<Double> from(double value) {
      return new AllianceRelative<>(value, AllianceFlipUtil::applyX);
    }

    public static AllianceRelative<Rotation2d> from(Rotation2d value) {
      return new AllianceRelative<>(value, AllianceFlipUtil::applyRotation);
    }

    public static AllianceRelative<Rotation3d> from(Rotation3d value) {
      return new AllianceRelative<>(value, AllianceFlipUtil::applyRotation);
    }

    public static AllianceRelative<Translation2d> from(Translation2d value) {
      return new AllianceRelative<>(value, AllianceFlipUtil::applyTranslation);
    }

    public static AllianceRelative<Translation3d> from(Translation3d value) {
      return new AllianceRelative<>(value, AllianceFlipUtil::applyTranslation);
    }

    public static AllianceRelative<Pose2d> from(Pose2d value) {
      return new AllianceRelative<>(value, AllianceFlipUtil::applyPose);
    }

    public static AllianceRelative<Pose3d> from(Pose3d value) {
      return new AllianceRelative<>(value, AllianceFlipUtil::applyPose);
    }
  }
}
