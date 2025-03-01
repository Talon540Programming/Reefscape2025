// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import java.nio.file.Path;
import java.util.Collections;
import java.util.List;
import lombok.Builder;

public class VisionConstants {

  //   public static final Map<Integer, Integer> blueAllianceAprilTagReefPoses = new HashMap<>();
  public static final int[] blueAllianceAprilTagReefPoses = new int[] {18, 19, 20, 21, 22, 17};

  //   static {
  //     blueAllianceAprilTagReefPoses.put(18, 0);
  //     blueAllianceAprilTagReefPoses.put(19, 1);
  //     blueAllianceAprilTagReefPoses.put(20, 2);
  //     blueAllianceAprilTagReefPoses.put(21, 3);
  //     blueAllianceAprilTagReefPoses.put(22, 4);
  //     blueAllianceAprilTagReefPoses.put(17, 5);
  //   }

  //   public static final Map<Integer, Integer> redAllianceAprilTagReefPoses = new HashMap<>();
  public static final int[] redAllianceAprilTagReefPoses = new int[] {7, 6, 11, 10, 9, 8};

  //   static {
  //     redAllianceAprilTagReefPoses.put(0,7);
  //     redAllianceAprilTagReefPoses.put(1,6);
  //     redAllianceAprilTagReefPoses.put(2,11);
  //     redAllianceAprilTagReefPoses.put(3,10);
  //     redAllianceAprilTagReefPoses.put(4,9);
  //     redAllianceAprilTagReefPoses.put(5,8);
  //   }

  public static final double ambiguityThreshold = 0.4;
  public static final double targetLogTimeSecs = 0.1;
  public static final double fieldBorderMargin = 0.5;
  public static final double zMargin = 0.75;
  public static final double xyStdDevCoefficient = 0.015;
  public static final double thetaStdDevCoefficient = 0.03;
  public static final double demoTagPosePersistenceSecs = 0.5;
  public static final double objDetectConfidenceThreshold = 0.8;

  public static final Translation2d branchPoseToScorePose =
      new Translation2d(Units.inchesToMeters(20), 0);
  public static final LoggedTunableNumber timestampOffset =
      new LoggedTunableNumber("AprilTagVision/TimestampOffset", 0.0);

  private static int monoExposure = 10000;
  private static int colorExposure = 10000;
  private static double monoGain = 0.3;
  private static double colorGain = 0.3;

  //   public static LoggedTunableNumber[] pitchAdjustments =
  //       switch (Constants.getRobot()) {
  //         case SIMBOT ->
  //             new LoggedTunableNumber[] {
  //               new LoggedTunableNumber("Vision/PitchAdjust0", 0.0),
  //               new LoggedTunableNumber("Vision/PitchAdjust1", 0.0)
  //             };
  //         case COMPBOT ->
  //             new LoggedTunableNumber[] {
  //               new LoggedTunableNumber("Vision/PitchAdjust0", 0.0),
  //               new LoggedTunableNumber("Vision/PitchAdjust1", 0.0),
  //               new LoggedTunableNumber("Vision/PitchAdjust2", 0.0),
  //               new LoggedTunableNumber("Vision/PitchAdjust3", 0.0)
  //             };
  //         default -> new LoggedTunableNumber[] {};
  //       };
  public static List<CameraConfig> cameras =
      switch (Constants.getRobot()) {
        case SIMBOT ->
            List.of(
                CameraConfig.builder()
                    .cameraName("frontleft")
                    .robotToCamera(
                        new Transform3d(
                            0.206,
                            0.272,
                            0.221,
                            new Rotation3d(0, Math.toRadians(-20), Math.toRadians(-35))))
                    .cameraBias(VecBuilder.fill(1.0, 1.0, 1.0))
                    .calibrationPath(
                        Path.of("camera_calibrations/mrcalibration_testcam@1280x800.json"))
                    .build(),
                CameraConfig.builder()
                    .cameraName("frontright")
                    .robotToCamera(
                        new Transform3d(
                            0.206,
                            -0.272,
                            0.221,
                            new Rotation3d(0, Math.toRadians(-20), Math.toRadians(35))))
                    .cameraBias(VecBuilder.fill(1.0, 1.0, 1.0))
                    .calibrationPath(
                        Path.of("camera_calibrations/mrcalibration_testcam@1280x800.json"))
                    .build()
                // ,
                // CameraConfig.builder()
                //     .cameraName("elevator")
                //     .robotToCamera(
                //         new Transform3d(
                //             0.05,
                //             0,
                //             0.783,
                //             new Rotation3d(0, Math.toRadians(-40), Math.toRadians(180))))
                //     .cameraBias(VecBuilder.fill(1.0, 1.0, 1.0))
                //     .calibrationPath(
                //         Path.of("camera_calibrations/mrcalibration_testcam@1280x800.json"))
                //     .build()
                );
        case COMPBOT ->
            List.of(
                CameraConfig.builder()
                    .cameraName("frontleft")
                    .robotToCamera(
                        new Transform3d(
                            0.206,
                            0.272,
                            0.221,
                            new Rotation3d(0, Math.toRadians(-20), Math.toRadians(-35))))
                    .cameraBias(VecBuilder.fill(1.0, 1.0, 1.0))
                    .calibrationPath(
                        Path.of("camera_calibrations/mrcalibration_testcam@1280x800.json"))
                    .build(),
                CameraConfig.builder()
                    .cameraName("frontright")
                    .robotToCamera(
                        new Transform3d(
                            0.206,
                            -0.272,
                            0.221,
                            new Rotation3d(0, Math.toRadians(-20), Math.toRadians(35))))
                    .cameraBias(VecBuilder.fill(1.0, 1.0, 1.0))
                    .calibrationPath(
                        Path.of("camera_calibrations/mrcalibration_testcam@1280x800.json"))
                    .build()
                //     ,
                // CameraConfig.builder()
                //     .cameraName("elevator")
                //     .robotToCamera(
                //         new Transform3d(
                //             0.05,
                //             0,
                //             0.783,
                //             new Rotation3d(0, Math.toRadians(-40), Math.toRadians(180))))
                //     .cameraBias(VecBuilder.fill(1.0, 1.0, 1.0))
                //     .calibrationPath(
                //         Path.of("camera_calibrations/mrcalibration_testcam@1280x800.json"))
                //     .build()
                );
        default -> Collections.<CameraConfig>emptyList();
      };

  @Builder
  public record CameraConfig(
      String cameraName,
      Transform3d robotToCamera,
      Matrix<N3, N1> cameraBias,
      Path calibrationPath) {}

  private VisionConstants() {}
}
