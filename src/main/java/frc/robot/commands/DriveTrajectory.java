// package frc.robot.commands;
//
// import choreo.trajectory.SwerveSample;
// import choreo.trajectory.Trajectory;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.PoseEstimator;
// import frc.robot.subsystems.drive.DriveBase;
// import frc.robot.util.AllianceFlipUtil;
// import frc.robot.util.LoggedTunableNumber;
// import java.util.Arrays;
// import org.littletonrobotics.junction.Logger;
//
// public class DriveTrajectory extends Command {
//   private static final LoggedTunableNumber linearkP =
//       new LoggedTunableNumber("DriveTrajectory/LinearkP");
//   private static final LoggedTunableNumber linearkD =
//       new LoggedTunableNumber("DriveTrajectory/LinearkD");
//   private static final LoggedTunableNumber thetakP =
//       new LoggedTunableNumber("DriveTrajectory/ThetakP");
//   private static final LoggedTunableNumber thetakD =
//       new LoggedTunableNumber("DriveTrajectory/thetakD");
//
//   static {
//     switch (Constants.getRobot()) {
//       case COMPBOT -> {
//         linearkP.initDefault(8.0);
//         linearkD.initDefault(0.0);
//         thetakP.initDefault(4.0);
//         thetakD.initDefault(0.0);
//       }
//       default -> {
//         linearkP.initDefault(4.0);
//         linearkD.initDefault(0.0);
//         thetakP.initDefault(12.0);
//         thetakD.initDefault(0.0);
//       }
//     }
//   }
//
//   private final PIDController xController = new PIDController(0.0, 0.0, 0.0);
//   private final PIDController yController = new PIDController(0.0, 0.0, 0.0);
//   private final PIDController thetaController = new PIDController(0.0, 0.0, 0.0);
//
//   private final DriveBase driveBase;
//   private final Timer timer = new Timer();
//   private final Trajectory<SwerveSample> trajectory;
//
//   public DriveTrajectory(DriveBase driveBase, Trajectory<SwerveSample> trajectory) {
//     this.driveBase = driveBase;
//     this.trajectory = trajectory;
//
//     thetaController.enableContinuousInput(-Math.PI, Math.PI);
//
//     addRequirements(driveBase);
//   }
//
//   @Override
//   public void initialize() {
//     timer.restart();
//
//     xController.reset();
//     yController.reset();
//     thetaController.reset();
//
//     Logger.recordOutput(
//         "Trajectory/TrajectoryPoses",
//
// Arrays.stream(trajectory.getPoses()).map(AllianceFlipUtil::apply).toArray(Pose2d[]::new));
//   }
//
//   @Override
//   public void execute() {
//     if (linearkP.hasChanged(hashCode()) || linearkD.hasChanged(hashCode())) {
//       xController.setPID(linearkP.get(), 0.0, linearkD.get());
//       yController.setPID(linearkP.get(), 0.0, linearkD.get());
//     }
//     if (thetakP.hasChanged(hashCode()) || thetakD.hasChanged(hashCode())) {
//       thetaController.setPID(thetakP.get(), 0.0, thetakD.get());
//     }
//
//     Pose2d robotPose = PoseEstimator.getInstance().getEstimatedPose();
//
//     SwerveSample sample = trajectory.sampleAt(timer.get(), AllianceFlipUtil.shouldFlip()).get();
//
//     driveBase.runVelocity(
//         ChassisSpeeds.fromFieldRelativeSpeeds(
//             sample.vx + xController.calculate(robotPose.getX(), sample.x),
//             sample.vy + yController.calculate(robotPose.getY(), sample.y),
//             sample.omega + thetaController.calculate(robotPose.getRotation().getRadians(),
// sample.heading),
//             robotPose.getRotation()
//         )
//     );
//   }
//
//   @Override
//   public boolean isFinished() {
//     return timer.get() >= trajectory.getTotalTime();
//   }
// }
