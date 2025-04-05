package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.TxTyObservation;
import frc.robot.RobotState.VisionObservation;
import frc.robot.subsystems.leds.LEDBase;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.VirtualSubsystem;
import java.util.*;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

@ExtensionMethod(GeomUtil.class)
public class VisionBase extends VirtualSubsystem {
  private static final double disconnectedTimeout = 0.5;
  public static final LoggedTunableNumber timestampOffset =
      new LoggedTunableNumber("AprilTagVision/TimestampOffset", 0.0);

  private final Alert aprilTagLayoutAlert = new Alert("", Alert.AlertType.kInfo);
  private static final LoggedNetworkBoolean aprilTagsReef =
      new LoggedNetworkBoolean("/SmartDashboard/AprilTagVision/OnlyUseReefAprilTags", false);
  private static final LoggedNetworkBoolean aprilTagFieldBorder =
      new LoggedNetworkBoolean("/SmartDashboard/AprilTagVision/OnlyUseBorderAprilTags", false);
  private FieldConstants.AprilTagLayoutType lastAprilTagLayout = null;

  private final VisionIO[] io;
  private final VisionIOInputs[] inputs;

  private final Timer[] disconnectedTimers;
  private final Alert[] cameraDisconnectedAlerts;

  private final Map<Integer, Double> lastFrameTimes = new HashMap<>();
  private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

  public VisionBase(VisionIO... camIOs) {
    io = camIOs;

    inputs = new VisionIOInputs[io.length];
    disconnectedTimers = new Timer[io.length];
    cameraDisconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputs();
      cameraDisconnectedAlerts[i] = new Alert("", Alert.AlertType.kError);
    }

    // Create map of last frame times for instances
    for (int i = 0; i < io.length; i++) {
      lastFrameTimes.put(i, 0.0);
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
          String.format("Vision/Inst%d (%s)", i, cameras[io[i].getCamIndex()].cameraName()),
          inputs[i]);
    }

    // Update disconnected alerts
    boolean anyNTDisconnected = false;
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
                    i, cameras[io[i].getCamIndex()].cameraName())
                : String.format(
                    "Vision Inst%d (%s) disconnected from NT",
                    i, cameras[io[i].getCamIndex()].cameraName()));
      }
      cameraDisconnectedAlerts[i].set(disconnected);
      anyNTDisconnected = anyNTDisconnected || !inputs[i].ntConnected;
    }
    LEDBase.getInstance().visionDisconnected = anyNTDisconnected;

    // Update FieldLayout
    var aprilTagType = getSelectedAprilTagLayout();
    if (aprilTagType != lastAprilTagLayout) {
      lastAprilTagLayout = aprilTagType;
      for (VisionIO visionIO : io) {
        visionIO.setAprilTagFieldLayout(aprilTagType);
      }
    }
    boolean atflAlertActive = aprilTagType != FieldConstants.defaultAprilTagType;
    aprilTagLayoutAlert.set(atflAlertActive);
    if (atflAlertActive) {
      aprilTagLayoutAlert.setText(
          "Non-default AprilTag layout in use (" + aprilTagType.toString() + ").");
    }

    // Loop over instances
    List<Pose3d> allCameraPoses = new ArrayList<>();
    List<Pose2d> allRobotPoses = new ArrayList<>();
    List<VisionObservation> allVisionObservations = new ArrayList<>();
    Map<Integer, TxTyObservation> allTxTyObservations = new HashMap<>();
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      int camConfigIndex = io[cameraIndex].getCamIndex();
      Map<Integer, TxTyObservation> txTyObservations = new HashMap<>();

      for (int observationIndex = 0;
          observationIndex < inputs[cameraIndex].observations.length;
          observationIndex++) {
        var observation = inputs[cameraIndex].observations[observationIndex];

        lastFrameTimes.put(cameraIndex, Timer.getTimestamp());

        if (!observation.hasResult) {
          continue;
        }

        var timestamp = observation.timestampSeconds + timestampOffset.get();

        // Switch based on detection technique
        Pose3d cameraPose = null;
        Pose2d robotPose = null;
        boolean useVisionRotation = false;
        Matrix<N3, N1> baseStdevs = null;
        if (observation.multitagResult.isPresent()) {
          var multitagRes = observation.multitagResult.get();
          // Handle Multitag
          cameraPose = multitagRes.multitagCamPose().toPose3d();
          robotPose =
              cameraPose
                  .toPose2d()
                  .transformBy(cameras[camConfigIndex].robotToCamera().toTransform2d().inverse());
          useVisionRotation = true;
          baseStdevs = multiTagStdevs;
        } else if (observation.singleTagResult.isPresent()) {
          var singleTagRes = observation.singleTagResult.get();

          double camToTagDistance;

          // Handle Single-tag
          if (singleTagRes.ambiguity() <= ambiguityThreshold) {
            // Ambiguity is good enough to disambiguate and use for global pose.
            var tagPoseOpt = aprilTagType.getLayout().getTagPose(singleTagRes.tagId());
            if (tagPoseOpt.isEmpty()) break;
            var tagPose = tagPoseOpt.get();

            var camToTag = singleTagRes.bestCamToTag();
            cameraPose = tagPose.transformBy(camToTag.inverse());
            robotPose =
                cameraPose
                    .toPose2d()
                    .transformBy(cameras[camConfigIndex].robotToCamera().toTransform2d().inverse());
            camToTagDistance = camToTag.getTranslation().getNorm();

            useVisionRotation = true;
            baseStdevs = singleTagStdevs;
          } else {
            // This estimation is shit, we can still use it for alignment though
            camToTagDistance = singleTagRes.bestCamToTag().getTranslation().getNorm();
          }

          // Handle TxTy Observation
          txTyObservations.put(
              singleTagRes.tagId(),
              new TxTyObservation(
                  singleTagRes.tagId(),
                  camConfigIndex,
                  singleTagRes.pitch(),
                  singleTagRes.yaw(),
                  camToTagDistance,
                  timestamp));
        }

        // Exit if no data
        if (cameraPose == null || robotPose == null) {
          continue;
        }

        // Exit if robot pose is off the field or outside reasonable bounds
        if (robotPose.getX() < -fieldBorderMargin
            || robotPose.getX() > FieldConstants.fieldLength + fieldBorderMargin
            || robotPose.getY() < -fieldBorderMargin
            || robotPose.getY() > FieldConstants.fieldWidth + fieldBorderMargin) {
          continue;
        }

        // Get tag poses and update last detection times
        List<Pose3d> tagPoses = new ArrayList<>();
        for (int tagId : observation.detectedTagIds) {
          lastTagDetectionTimes.put(tagId, Timer.getTimestamp());
          aprilTagType.getLayout().getTagPose(tagId).ifPresent(tagPoses::add);
        }

        // Calculate average distance to tag
        double totalDistance = 0.0;
        for (Pose3d tagPose : tagPoses) {
          totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
        }
        double avgDistance = totalDistance / tagPoses.size();

        // Update observation trust matrix
        double xyStdev =
            Math.pow(avgDistance, 2) / tagPoses.size() * cameras[camConfigIndex].cameraBiasScalar();
        double rotStdev =
            useVisionRotation
                ? Math.pow(avgDistance, 2)
                    / tagPoses.size()
                    * cameras[camConfigIndex].cameraBiasScalar()
                : Double.POSITIVE_INFINITY;
        var observationStdevs =
            baseStdevs.elementTimes(VecBuilder.fill(xyStdev, xyStdev, rotStdev));

        allCameraPoses.add(cameraPose);
        allRobotPoses.add(robotPose);
        allVisionObservations.add(new VisionObservation(robotPose, timestamp, observationStdevs));

        // Log data from instance
        if (enableInstanceLogging) {
          Logger.recordOutput(
              "AprilTagVision/Inst" + cameraIndex + "/LatencySecs",
              Timer.getTimestamp() - timestamp);
          Logger.recordOutput("AprilTagVision/Inst" + cameraIndex + "/CameraPose", cameraPose);
          Logger.recordOutput("AprilTagVision/Inst" + cameraIndex + "/RobotPose", robotPose);
          Logger.recordOutput(
              "AprilTagVision/Inst" + cameraIndex + "/TagPoses", tagPoses.toArray(Pose3d[]::new));
        }
      }

      // Save tx ty observation data
      for (var observation : txTyObservations.values()) {
        if (!allTxTyObservations.containsKey(observation.tagId())
            || observation.distance() < allTxTyObservations.get(observation.tagId()).distance()) {
          allTxTyObservations.put(observation.tagId(), observation);
        }
      }

      //   If no frames from instances, clear robot pose
      if (enableInstanceLogging && inputs[cameraIndex].observations.length == 0) {
        Logger.recordOutput("AprilTagVision/Inst" + cameraIndex + "/CameraPose", Pose3d.kZero);
        Logger.recordOutput("AprilTagVision/Inst" + cameraIndex + "/RobotPose", Pose2d.kZero);
      }

      // If no recent frames from instance, clear tag poses
      if (enableInstanceLogging
          && Timer.getTimestamp() - lastFrameTimes.get(cameraIndex) > targetLogTimeSecs) {
        Logger.recordOutput("AprilTagVision/Inst" + cameraIndex + "/TagPoses", new Pose3d[] {});
      }
    }

    // Log poses
    Logger.recordOutput("AprilTagVision/CameraPoses", allCameraPoses.toArray(Pose3d[]::new));
    Logger.recordOutput("AprilTagVision/RobotPoses", allRobotPoses.toArray(Pose2d[]::new));

    // Log tag poses
    List<Pose3d> allTagPoses = new ArrayList<>();
    for (Map.Entry<Integer, Double> detectionEntry : lastTagDetectionTimes.entrySet()) {
      if (Timer.getTimestamp() - detectionEntry.getValue() < targetLogTimeSecs) {
        aprilTagType.getLayout().getTagPose(detectionEntry.getKey()).ifPresent(allTagPoses::add);
      }
    }
    Logger.recordOutput("AprilTagVision/TagPoses", allTagPoses.toArray(Pose3d[]::new));

    // Send results to robot state
    allVisionObservations.stream()
        .sorted(Comparator.comparingDouble(VisionObservation::timestamp))
        .forEach(RobotState.getInstance()::addVisionObservation);
    allTxTyObservations.values().forEach(RobotState.getInstance()::addTxTyObservation);
  }

  /** Returns the current AprilTag layout type. */
  public static FieldConstants.AprilTagLayoutType getSelectedAprilTagLayout() {
    if (aprilTagsReef.get()) {
      if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
        return FieldConstants.AprilTagLayoutType.BLUE_REEF;
      } else {
        return FieldConstants.AprilTagLayoutType.RED_REEF;
      }
    } else if (aprilTagFieldBorder.get()) {
      return FieldConstants.AprilTagLayoutType.FIELD_BORDER;
    } else {
      return FieldConstants.defaultAprilTagType;
    }
  }
}
