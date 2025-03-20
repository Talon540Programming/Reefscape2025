package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.PoseEstimator;
import frc.robot.PoseEstimator.VisionObservation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

  private final VisionIO[] cameras;
  private final VisionIOInputsAutoLogged[] cameraInputs;

  public Vision(VisionIO... cameras) {
    this.cameras = cameras;
    cameraInputs = new VisionIOInputsAutoLogged[this.cameras.length];
    for (int i = 0; i < this.cameras.length; i++) {
      cameraInputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < cameras.length; i++) {
      var input = cameraInputs[i];

      cameras[i].updateInputs(input);
      Logger.processInputs("Vision/Cam" + i, input);

      // Don't report if there is no valid global pose estimate
      if (!input.hasResult) continue;

      PoseEstimator.getInstance()
          .addVisionObservation(
              new VisionObservation(
                  input.estimatedRobotPose.toPose2d(),
                  input.timestampSeconds,
                  input.visionMeasurementStdDevs));
    }
  }

  @AutoLogOutput(key = "Vision/NearestReefPose")
  public Pose2d getNearestReefFace() {
    for (int i = 0; i < cameras.length; i++) {
      var input = cameraInputs[i];
      if (input.detectedTagsIds.length == 0) continue;
      for (int j = 0; j < input.detectedTagsIds.length; j++) {
        for (int k = 0; k < VisionConstants.reefAprilTags.length; k++) {
          if (input.detectedTagsIds[j] == VisionConstants.reefAprilTags[k]
              && input.tagDistances[j] <= maxReefTagDistance) {
            // Logger.recordOutput("Vision/ReefFacePose", FieldConstants.Reef.centerFaces[k]);
            return FieldConstants.Reef.centerFaces[k];
          }
        }
      }
    }

    return PoseEstimator.getInstance().getEstimatedPose();
  }

  @AutoLogOutput(key = "Vision/NearestLeftBranch")
  public Pose2d getNearestLeftBranch() {
    return getNearestReefFace().plus(FieldConstants.Reef.centerToLeftBranch);
  }

  @AutoLogOutput(key = "Vision/NearestRightBranch")
  public Pose2d getNearestRightBranch() {
    return getNearestReefFace().plus(FieldConstants.Reef.centerToRightBranch);
  }
}
