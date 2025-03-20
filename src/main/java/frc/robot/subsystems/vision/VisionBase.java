package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.FieldConstants;
import frc.robot.PoseEstimator;
import frc.robot.PoseEstimator.VisionObservation;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.VirtualSubsystem;
import java.util.*;
import org.littletonrobotics.junction.Logger;

public class VisionBase extends VirtualSubsystem {
  private static final double disconnectedTimeout = 0.5;
  public static final LoggedTunableNumber timestampOffset =
      new LoggedTunableNumber("AprilTagVision/TimestampOffset", 0.0);

  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;

  private final Timer[] disconnectedTimers;
  private final Alert[] cameraDisconnectedAlerts;

  private final Map<Integer, Double> lastFrameTimes = new HashMap<>();
  private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

  public VisionBase(VisionIO... camIOs) {
    io = camIOs;

    inputs = new VisionIOInputsAutoLogged[io.length];
    disconnectedTimers = new Timer[io.length];
    cameraDisconnectedAlerts = new Alert[io.length];

    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
      cameraDisconnectedAlerts[i] = new Alert("", Alert.AlertType.kError);
    }

    for (int i = 0; i < io.length; i++) {
      disconnectedTimers[i] = new Timer();
      disconnectedTimers[i].start();
    }
  }

  @Override
  public void periodic() {
    // Update Inputs
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs(
          String.format("Vision/Inst%d (%s)", i, cameras[i].cameraName()), inputs[i]);
    }

    // TODO Update LEDs based on vision state
    // Update disconnected alerts
    for (int i = 0; i < io.length; i++) {
      if (inputs[i].observations.length > 0) {
        disconnectedTimers[i].reset();
      }

      boolean disconnected =
          disconnectedTimers[i].hasElapsed(disconnectedTimeout) || !inputs[i].ntConnected;
      if (disconnected) {
        cameraDisconnectedAlerts[i].setText(
            inputs[i].ntConnected
                ? String.format(
                    "Vision Inst%d (%s) connected to NT but not publishing frames",
                    i, cameras[i].cameraName())
                : String.format(
                    "Vision Inst%d (%s) disconnected from NT", i, cameras[i].cameraName()));
      }
      cameraDisconnectedAlerts[i].set(disconnected);
    }

    // Loop over instances
    List<Pose2d> allRobotPoses = new ArrayList<>();
    List<VisionObservation> allVisionObservations = new ArrayList<>();
    // Map<Integer, TxTyObservation> allTxTyObservations = new HashMap<>();
    for (int i = 0; i < io.length; i++) {
      for (var observation : inputs[i].observations) {
        var timestamp = observation.timestamp() + timestampOffset.get();

        // Switch based on detection technique
        Pose3d cameraPose = null;
        Pose2d robotPose = null;
        boolean useVisionRotation = false;
        Matrix<N3, N1> baseStdevs = null;

        if (observation.multitagTagToCamera() != null) {
          // Handle Multitag
          cameraPose = GeomUtil.toPose3d(observation.multitagTagToCamera());
          robotPose =
              cameraPose
                  .toPose2d()
                  .transformBy(GeomUtil.toTransform2d(cameras[i].robotToCamera()).inverse());
          useVisionRotation = true;
          baseStdevs = multiTagStdevs;
        } else if (observation.bestTagToCamera() != null) {
          // Handle Singletag
          if (observation.ambiguity() > ambiguityThreshold) {
            continue;
          }

          var bestCamPose = GeomUtil.toPose3d(observation.bestTagToCamera());
          var altCamPose = GeomUtil.toPose3d(observation.altTagToCamera());

          Transform2d cameraToRobot = GeomUtil.toTransform2d(cameras[i].robotToCamera()).inverse();
          var bestPose = bestCamPose.toPose2d().transformBy(cameraToRobot);
          var altPose = altCamPose.toPose2d().transformBy(cameraToRobot);

          // TODO latency compensate this to get rotation at this timestamp
          var currentRot = PoseEstimator.getInstance().getRotation();
          var bestRot = bestPose.getRotation();
          var altRot = altPose.getRotation();

          if (Math.abs(currentRot.minus(bestRot).getRadians())
              < Math.abs(currentRot.minus(altRot).getRadians())) {
            cameraPose = bestCamPose;
            robotPose = bestPose;
          } else {
            cameraPose = altCamPose;
            robotPose = altPose;
          }

          useVisionRotation = false;
          baseStdevs = singleTagStdevs;
        }

        // Exit if no data
        if (cameraPose == null || robotPose == null) {
          continue;
        }

        // Exit if robot pose is off the field or outside of reasonable bounds
        if (robotPose.getX() < -fieldBorderMargin
            || robotPose.getX() > FieldConstants.fieldLength + fieldBorderMargin
            || robotPose.getY() < -fieldBorderMargin
            || robotPose.getY() > FieldConstants.fieldWidth + fieldBorderMargin) {
          continue;
        }

        // Get tag poses and update last detection times
        List<Pose3d> tagPoses = new ArrayList<>();
        for (int tagId : observation.detectedTagsIds()) {
          lastTagDetectionTimes.put(tagId, Timer.getTimestamp());
          FieldConstants.fieldLayout.getTagPose(tagId).ifPresent(tagPoses::add);
        }

        // Calculate average distance to tag
        double totalDistance = 0.0;
        for (Pose3d tagPose : tagPoses) {
          totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
        }
        double avgDistance = totalDistance / tagPoses.size();

        // Update observation trust matrix
        double xyStdev = Math.pow(avgDistance, 2) / tagPoses.size() * cameras[i].cameraBiasScalar();
        double rotStdev =
            useVisionRotation
                ? Math.pow(avgDistance, 2) / tagPoses.size() * cameras[i].cameraBiasScalar()
                : Double.POSITIVE_INFINITY;
        var observationStdevs =
            baseStdevs.elementTimes(VecBuilder.fill(xyStdev, xyStdev, rotStdev));

        allVisionObservations.add(new VisionObservation(robotPose, timestamp, observationStdevs));
        allRobotPoses.add(robotPose);

        // Log data from instance
        if (enableInstanceLogging) {
          Logger.recordOutput(
              "AprilTagVision/Inst" + i + "/LatencySecs", Timer.getTimestamp() - timestamp);
          Logger.recordOutput("AprilTagVision/Inst" + i + "/RobotPose", robotPose);
          Logger.recordOutput(
              "AprilTagVision/Inst" + i + "/TagPoses", tagPoses.toArray(Pose3d[]::new));
        }
      }

      // TODO handle TxTy
      // TODO figure out if we want to figure out local pose estimation
      // // Get tag tx ty observation data
      // Map<Integer, TxTyObservation> txTyObservations = new HashMap<>();
      // for(var observation : inputs[i].observations) {
      //   var timestamp = observation.timestamp() + timestampOffset.get();
      // }

      // If no frames from instances, clear robot pose
      if (enableInstanceLogging && inputs[i].observations.length == 0) {
        Logger.recordOutput("AprilTagVision/Inst" + i + "/RobotPose", Pose2d.kZero);
      }

      // If no recent frames from instance, clear tag poses
      if (enableInstanceLogging
          && Timer.getTimestamp() - lastFrameTimes.get(i) > targetLogTimeSecs) {
        Logger.recordOutput("AprilTagVision/Inst" + i + "/TagPoses", new Pose3d[] {});
      }
    }

    // Log robot poses
    Logger.recordOutput("AprilTagVision/RobotPoses", allRobotPoses.toArray(Pose2d[]::new));

    // Log tag poses
    List<Pose3d> allTagPoses = new ArrayList<>();
    for (Map.Entry<Integer, Double> detectionEntry : lastTagDetectionTimes.entrySet()) {
      if (Timer.getTimestamp() - detectionEntry.getValue() < targetLogTimeSecs) {
        FieldConstants.fieldLayout.getTagPose(detectionEntry.getKey()).ifPresent(allTagPoses::add);
      }
    }
    Logger.recordOutput("AprilTagVision/TagPoses", allTagPoses.toArray(Pose3d[]::new));

    // Send results to robot state
    allVisionObservations.stream()
        .sorted(Comparator.comparingDouble(VisionObservation::timestamp))
        .forEach(PoseEstimator.getInstance()::addVisionObservation);
    // allTxTyObservations.values().stream().forEach(PoseEstimator.getInstance()::addTxTyObservation);
  }
}
