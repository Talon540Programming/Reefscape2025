// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import java.nio.file.Path;
import lombok.Builder;

class VisionConstants {
  private static final boolean forceEnableInstanceLogging = true;
  public static final boolean enableInstanceLogging =
      forceEnableInstanceLogging || Constants.getMode() == Constants.Mode.REPLAY;

  public static final double ambiguityThreshold = 0.4;
  public static final double targetLogTimeSecs = 0.1;
  public static final double fieldBorderMargin = 0.5;

  public static final Vector<N3> singleTagStdevs =
      VecBuilder.fill(0.1, 0.1, Math.toRadians(5.0)); // TODO
  public static final Vector<N3> multiTagStdevs =
      VecBuilder.fill(0.015, 0.015, Math.toRadians(2.0)); // TODO

  public static CameraConfig[] cameras =
      switch (Constants.getRobot()) {
        case COMPBOT, SIMBOT ->
            new CameraConfig[] {
              CameraConfig.builder()
                  .cameraName("frontleft")
                  .robotToCamera(
                      new Transform3d(
                          0.206,
                          0.272,
                          0.221,
                          new Rotation3d(0, Math.toRadians(-20), Math.toRadians(-35))))
                  .cameraBiasScalar(1.0)
                  .calibrationPath(
                      Path.of("camera_calibrations/frontleft_calibration@1280x800.json"))
                  .build(),
              CameraConfig.builder()
                  .cameraName("frontright")
                  .robotToCamera(
                      new Transform3d(
                          0.206,
                          -0.272,
                          0.221,
                          new Rotation3d(0, Math.toRadians(-20), Math.toRadians(35))))
                  .cameraBiasScalar(1.0)
                  .calibrationPath(
                      Path.of("camera_calibrations/frontright_calibration@1280x800.json"))
                  .build()
            };
        default -> new CameraConfig[] {};
      };

  @Builder
  public record CameraConfig(
      String cameraName,
      Transform3d robotToCamera,
      double cameraBiasScalar,
      Path calibrationPath) {}
}
