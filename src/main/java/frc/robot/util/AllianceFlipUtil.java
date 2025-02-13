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
    return new Translation2d(applyX(translation2d.getX()), translation2d.getY());
  }

  private static Translation3d applyTranslation(Translation3d translation3d) {
    return new Translation3d(
        applyX(translation3d.getX()), translation3d.getY(), translation3d.getZ());
  }

  private static Rotation2d applyRotation(Rotation2d rotation) {
    return new Rotation2d(-rotation.getCos(), rotation.getSin());
  }

  private static Pose2d applyPose(Pose2d pose) {
    return new Pose2d(applyTranslation(pose.getTranslation()), applyRotation(pose.getRotation()));
  }

  private static boolean shouldFlip() {
    var currentAllianceOpt = DriverStation.getAlliance();
    return currentAllianceOpt.isPresent() && currentAllianceOpt.get() == DriverStation.Alliance.Red;
  }

  public static double apply(double x) {
    return shouldFlip() ? applyX(x) : x;
  }

  public static Translation2d apply(Translation2d translation) {
    return shouldFlip() ? applyTranslation(translation) : translation;
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? applyRotation(rotation) : rotation;
  }

  public static Pose2d apply(Pose2d pose) {
    return shouldFlip() ? applyPose(pose) : pose;
  }

  public static Translation3d apply(Translation3d translation3d) {
    return shouldFlip() ? applyTranslation(translation3d) : translation3d;
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

    public static AllianceRelative<Translation2d> from(Translation2d value) {
      return new AllianceRelative<>(value, AllianceFlipUtil::applyTranslation);
    }

    public static AllianceRelative<Pose2d> from(Pose2d value) {
      return new AllianceRelative<>(value, AllianceFlipUtil::applyPose);
    }

    public static AllianceRelative<Translation3d> from(Translation3d value) {
      return new AllianceRelative<>(value, AllianceFlipUtil::applyTranslation);
    }
  }
}
