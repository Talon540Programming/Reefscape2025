package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ElevatorMechanismVisualizer implements AutoCloseable {
  private final String mechanismName;
  private final String logKey;
  private final double elevatorLength;
  private final Pose3d originOfExtension;

  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d mechanismLigament;

  public ElevatorMechanismVisualizer(
      String mechanismName,
      String logKey,
      double elevatorLength,
      Pose3d originOfExtension,
      double mechanismCanvasWidthMeters,
      double mechanismCanvasHeightMeters) {
    this.mechanismName = mechanismName;
    this.logKey = logKey;
    this.elevatorLength = elevatorLength;

    this.originOfExtension = originOfExtension;

    this.mechanism =
        new LoggedMechanism2d(
            mechanismCanvasWidthMeters, mechanismCanvasHeightMeters, new Color8Bit(14, 17, 23));
    LoggedMechanismRoot2d extender =
        this.mechanism.getRoot("ElevatorOrigin", 1, originOfExtension.getZ());

    mechanismLigament =
        extender.append(
            new LoggedMechanismLigament2d(
                "elevator", elevatorLength, 84.5, 4, new Color8Bit(Color.kBlue)));
  }

  public void update(double mechanismExtension) {
    mechanismLigament.setLength(elevatorLength + mechanismExtension);

    Logger.recordOutput(mechanismName + "/Mechanism2d/" + logKey, mechanism);

    Logger.recordOutput(
        mechanismName + "/Mechanism3d/" + logKey,
        new Pose3d(originOfExtension.getTranslation(), new Rotation3d(0, 0, 0)));
  }

  @Override
  public void close() {
    mechanismLigament.close();
    mechanism.close();
  }
}
