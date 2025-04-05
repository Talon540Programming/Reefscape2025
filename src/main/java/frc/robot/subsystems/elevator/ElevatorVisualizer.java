package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

class ElevatorVisualizer {
  private final String key;
  private final LoggedMechanism2d mechanism =
      new LoggedMechanism2d(
          Units.inchesToMeters(28.0), Units.feetToMeters(7.0), new Color8Bit(Color.kDarkGray));
  private final LoggedMechanismLigament2d elevatorMechanism;

  public ElevatorVisualizer(String key) {
    this.key = key;

    LoggedMechanismRoot2d root =
        mechanism.getRoot(key + " Root", elevatorOrigin2d.getX(), elevatorOrigin2d.getY());
    elevatorMechanism =
        root.append(
            new LoggedMechanismLigament2d(
                key + " Elevator",
                Units.inchesToMeters(26.0),
                elevatorAngle.getDegrees(),
                4.0,
                new Color8Bit(Color.kFirstBlue)));
  }

  public void update(double elevatorPositionMeters) {
    // final double heightFromBottom =
    //     elevatorPositionMeters + dispenserToBottom + dispenserToTop + stageThickness * 2.0;
    // final double firstStageHeight =
    //     Math.max(
    //         heightFromBottom - firstStageHeight - stageThickness,
    //         stageThickness);
    // final double secondStageHeight =
    //     Math.max(firstStageHeight - stageHeight + stageToStage - stageThickness, 0.0);
    //

    var robotPose = RobotState.getInstance().getEstimatedPose();

    Logger.recordOutput(
        "Mechanism3d/" + key + "/Superstructure",
        new Pose3d(
                robotPose.getX(),
                robotPose.getY(),
                0,
                new Rotation3d(0.0, 0.0, robotPose.getRotation().getRadians()))
            .transformBy(new Transform3d(dispenserOrigin3d, Rotation3d.kZero))
            .transformBy(
                new Transform3d(
                    new Translation3d(
                        elevatorPositionMeters,
                        new Rotation3d(0.0, -elevatorAngle.getRadians(), 0.0)),
                    Rotation3d.kZero)));
  }
}
