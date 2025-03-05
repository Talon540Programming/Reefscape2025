package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PoseEstimator;
import frc.robot.PoseEstimator.VisionObservation;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

  private final VisionIO[] cameras;
  private final VisionIOInputs[] cameraInputs;

  public Vision(VisionIO... cameras) {
    this.cameras = cameras;
    cameraInputs = new VisionIOInputs[this.cameras.length];
    for (int i = 0; i < this.cameras.length; i++) {
      cameraInputs[i] = new VisionIOInputs();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < cameras.length; i++) {
      var input = cameraInputs[i];

      cameras[i].updateInputs(input);
      Logger.processInputs("Vision/Cam" + i, input);

      // Don't report if there is no valid global pose estimate

      PoseEstimator.getInstance()
          .addVisionObservation(
              new VisionObservation(
                  input.estimatedRobotPose.toPose2d(),
                  input.timestampSeconds,
                  input.visionMeasurementStdDevs));
    }
  }
}
