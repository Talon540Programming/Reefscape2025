// package frc.robot.commands;

// import choreo.auto.AutoFactory;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.PoseEstimator;
// import frc.robot.subsystems.dispenser.DispenserBase;
// import frc.robot.subsystems.drive.DriveBase;
// import frc.robot.subsystems.elevator.ElevatorBase;
// import frc.robot.subsystems.elevator.ElevatorState;
// import frc.robot.subsystems.intake.IntakeBase;
// import frc.robot.util.AllianceFlipUtil;

// public class AutoRoutine {
//   private final DriveBase drive;
//   private final ElevatorBase elevator;
//   private final DispenserBase dispenser;
//   private final IntakeBase intake;

//   private final AutoFactory autoFactory;
//   private final DriveTrajectory trajectoryFollower;

//   public AutoRoutine(
//       DriveBase drive, ElevatorBase elevator, DispenserBase dispenser, IntakeBase intake) {
//     this.drive = drive;
//     this.elevator = elevator;
//     this.dispenser = dispenser;
//     this.intake = intake;

//     trajectoryFollower = new DriveTrajectory(drive);

//     autoFactory =
//         new AutoFactory(
//             PoseEstimator.getInstance()::getEstimatedPose,
//             PoseEstimator.getInstance()::resetPose,
//             trajectoryFollower::followTrajectory,
//             AllianceFlipUtil.shouldFlip(),
//             drive);
//   }

//   public Command TaxiL2() {
//     return Commands.sequence(
//         autoFactory.resetOdometry("taxi"),
//         Commands.deadline(
//             autoFactory.trajectoryCmd("taxi"),
//             Commands.runOnce(() -> elevator.setGoal(ElevatorState.L2_CORAL))
//                 .andThen(dispenser.eject(() -> ElevatorState.L2_CORAL))));
//   }
// }

