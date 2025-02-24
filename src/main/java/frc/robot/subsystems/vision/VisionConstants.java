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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import java.nio.file.Path;
import java.util.Collections;
import java.util.List;
import lombok.Builder;

public class VisionConstants {
  public static final double ambiguityThreshold = 0.4;
  public static final double targetLogTimeSecs = 0.1;
  public static final double fieldBorderMargin = 0.5;
  public static final double zMargin = 0.75;
  public static final double xyStdDevCoefficient = 0.015;
  public static final double thetaStdDevCoefficient = 0.03;
  public static final double demoTagPosePersistenceSecs = 0.5;
  public static final double objDetectConfidenceThreshold = 0.8;
  public static final LoggedTunableNumber timestampOffset =
      new LoggedTunableNumber("AprilTagVision/TimestampOffset", 0.0);

  private static int monoExposure = 10000;
  private static int colorExposure = 10000;
  private static double monoGain = 0.3;
  private static double colorGain = 0.3;

  public static LoggedTunableNumber[] pitchAdjustments =
      switch (Constants.getRobot()) {
        case SIMBOT ->
            new LoggedTunableNumber[] {
              new LoggedTunableNumber("Vision/PitchAdjust0", 0.0),
              new LoggedTunableNumber("Vision/PitchAdjust1", 0.0)
            };
        case COMPBOT ->
            new LoggedTunableNumber[] {
              new LoggedTunableNumber("Vision/PitchAdjust0", 0.0),
              new LoggedTunableNumber("Vision/PitchAdjust1", 0.0),
              new LoggedTunableNumber("Vision/PitchAdjust2", 0.0),
              new LoggedTunableNumber("Vision/PitchAdjust3", 0.0)
            };
        default -> new LoggedTunableNumber[] {};
      };
  public static List<CameraConfig> cameras =
      switch (Constants.getRobot()) {
        case SIMBOT ->
            List.of(
                CameraConfig.builder()
                    .cameraName("frontleft")
                    .robotToCamera(
                        new Transform3d(
                            0.247316498,
                            0.21199348,
                            0.2159,
                            new Rotation3d(0, Math.toRadians(-15), Math.toRadians(-25))))
                    .cameraBias(VecBuilder.fill(1.0, 1.0, 1.0)) // TODO
                    .calibrationPath(
                        Path.of("camera_calibrations/mrcalibration_testcam@1280x800.json"))
                    .build(),
                CameraConfig.builder()
                    .cameraName("frontright")
                    .robotToCamera(
                        new Transform3d(
                            0.247316498,
                            -0.21199348,
                            0.2159,
                            new Rotation3d(0, Math.toRadians(-15), Math.toRadians(25))))
                    .cameraBias(VecBuilder.fill(1.0, 1.0, 1.0)) // TODO
                    .calibrationPath(
                        Path.of("camera_calibrations/mrcalibration_testcam@1280x800.json"))
                    .build()
                // ,
                // CameraConfig.builder()
                //     .cameraName("testcam2")
                //     .robotToCamera(
                //         new Transform3d(
                //             -0.330312,
                //             0.138773,
                //             0.157061,
                //             new Rotation3d(0, Math.toRadians(-30), Math.PI)))
                //     .cameraBias(VecBuilder.fill(1.0, 1.0, 1.0)) // TODO
                //     .calibrationPath(
                //         Path.of(

                // "camera_calibrations/mrcalibration_testcam2@800x600.json"))
                //     .build()
                );
        case COMPBOT ->
            List.of(
                CameraConfig.builder()
                    .cameraName("UNDER_SHOOTER")
                    .robotToCamera(
                        new Transform3d(
                            -0.330312,
                            0.138773,
                            0.157061,
                            new Rotation3d(0, Math.toRadians(-30), Math.PI)))
                    .cameraBias(VecBuilder.fill(1.0, 1.0, 1.0)) // TODO
                    .calibrationPath(
                        Path.of(
                            "camera_calibrations/photon_calibration_UNDER_SHOOTER_1280x720.json"))
                    .build(),
                CameraConfig.builder()
                    .cameraName("BACK_LEFT")
                    .robotToCamera(
                        new Transform3d(
                            -0.285206,
                            0.283806,
                            0.272624,
                            new Rotation3d(0, Math.toRadians(-20), Math.toRadians(135))))
                    .cameraBias(VecBuilder.fill(1.0, 1.0, 1.0)) // TODO
                    .calibrationPath(
                        Path.of("camera_calibrations/photon_calibration_BACK_LEFT_1280x720.json"))
                    .build(),
                CameraConfig.builder()
                    .cameraName("FRONT_LEFT")
                    .robotToCamera(
                        new Transform3d(
                            0.278740,
                            0.280968,
                            0.272098,
                            new Rotation3d(0, Math.toRadians(-45), Math.toRadians(-22.5))))
                    .cameraBias(VecBuilder.fill(1.0, 1.0, 1.0)) // TODO
                    .calibrationPath(
                        Path.of("camera_calibrations/photon_calibration_FRONT_LEFT_1280x720.json"))
                    .build(),
                CameraConfig.builder()
                    .cameraName("FRONT_RIGHT")
                    .robotToCamera(
                        new Transform3d(
                            0.281034,
                            -0.278751,
                            0.272098,
                            new Rotation3d(0, Math.toRadians(-45), Math.toRadians(67.5))))
                    .cameraBias(VecBuilder.fill(1.0, 1.0, 1.0)) // TODO
                    .calibrationPath(
                        Path.of("camera_calibrations/photon_calibration_FRONT_RIGHT_1280x720.json"))
                    .build());
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
