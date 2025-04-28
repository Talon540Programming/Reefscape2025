package frc.robot.commands;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.MirrorUtil;
import java.util.Arrays;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveTrajectory extends Command {
  private static final LoggedTunableNumber linearkP =
      new LoggedTunableNumber("DriveTrajectory/LinearkP");
  private static final LoggedTunableNumber linearkD =
      new LoggedTunableNumber("DriveTrajectory/LinearkD");
  private static final LoggedTunableNumber thetakP =
      new LoggedTunableNumber("DriveTrajectory/ThetakP");
  private static final LoggedTunableNumber thetakD =
      new LoggedTunableNumber("DriveTrajectory/ThetakD");

  static {
    switch (Constants.getRobot()) {
      case COMPBOT -> {
        linearkP.initDefault(8.0);
        linearkD.initDefault(0.0);
        thetakP.initDefault(4.0);
        thetakD.initDefault(0.0);
      }
      default -> {
        linearkP.initDefault(4.0);
        linearkD.initDefault(0.0);
        thetakP.initDefault(12.0);
        thetakD.initDefault(0.0);
      }
    }
  }

  private final DriveBase driveBase;
  private final Trajectory<SwerveSample> trajectory;
  private final Timer timer = new Timer();
  private final Supplier<Pose2d> robotPose;
  private final boolean respectMirror;

  private final PIDController xController = new PIDController(0.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(0.0, 0.0, 0.0);
  private final PIDController thetaController = new PIDController(0.0, 0.0, 0.0);

  public DriveTrajectory(
      DriveBase driveBase,
      Trajectory<SwerveSample> trajectory,
      Supplier<Pose2d> robot,
      boolean shouldMirror) {
    this.driveBase = driveBase;
    this.trajectory = trajectory;
    this.robotPose = robot;
    this.respectMirror = shouldMirror;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(driveBase);
  }

  public DriveTrajectory(
      DriveBase driveBase, Trajectory<SwerveSample> trajectory, boolean respectMirror) {
    this(driveBase, trajectory, RobotState.getInstance()::getEstimatedPose, respectMirror);
  }

  public DriveTrajectory(DriveBase driveBase, Trajectory<SwerveSample> trajectory) {
    this(driveBase, trajectory, false);
  }

  @Override
  public void initialize() {
    timer.restart();

    xController.reset();
    yController.reset();
    thetaController.reset();

    Logger.recordOutput(
        "Trajectory/TrajectoryPoses",
        Arrays.stream(trajectory.getPoses())
            .map(pose -> AllianceFlipUtil.apply(respectMirror ? MirrorUtil.apply(pose) : pose))
            .toArray(Pose2d[]::new));
  }

  @Override
  public void execute() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          xController.setPID(linearkP.get(), 0.0, linearkD.get());
          yController.setPID(linearkP.get(), 0.0, linearkD.get());
        },
        true,
        linearkP,
        linearkD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> thetaController.setPID(thetakP.get(), 0.0, thetakD.get()),
        true,
        thetakP,
        thetakD);

    var robot = robotPose.get();
    var trajSample = trajectory.sampleAt(timer.get(), false).orElseThrow();
    var setpointTrajSample =
        AllianceFlipUtil.apply(respectMirror ? MirrorUtil.apply(trajSample) : trajSample);

    double xFeedback = xController.calculate(robot.getX(), setpointTrajSample.x);
    double yFeedback = yController.calculate(robot.getY(), setpointTrajSample.y);
    double thetaFeedback =
        thetaController.calculate(robot.getRotation().getRadians(), setpointTrajSample.heading);

    // Command drive
    driveBase.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            setpointTrajSample.vx + xFeedback,
            setpointTrajSample.vy + yFeedback,
            setpointTrajSample.omega + thetaFeedback,
            robot.getRotation()));

    // Log outputs
    Logger.recordOutput("Trajectory/RobotPose", robot);
    Logger.recordOutput("Trajectory/SetpointPose", setpointTrajSample);
    Logger.recordOutput(
        "Trajectory/Feedback",
        new Pose2d(xFeedback, yFeedback, Rotation2d.fromRadians(thetaFeedback)));
    Logger.recordOutput(
        "Trajectory/VelocityFeedforward",
        new ChassisSpeeds(setpointTrajSample.vx, setpointTrajSample.vy, setpointTrajSample.omega));
  }

  @Override
  public boolean isFinished() {
    return timer.get() >= trajectory.getTotalTime();
  }
}
