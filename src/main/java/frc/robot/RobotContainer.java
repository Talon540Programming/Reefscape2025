package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.dispenser.DispenserBase;
import frc.robot.subsystems.dispenser.DispenserIO;
import frc.robot.subsystems.dispenser.DispenserIOSim;
import frc.robot.subsystems.dispenser.DispenserIOSpark;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.intake.IntakeBase;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSpark;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Load PoseEstimator class
  private final PoseEstimator poseEstimator = PoseEstimator.getInstance();

  // Subsystems
  private final DriveBase driveBase;
  private final IntakeBase intakeBase;
  private final ElevatorBase elevatorBase;
  private final DispenserBase dispenserBase;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    switch (Constants.getMode()) {
      case REAL -> {
        driveBase =
            new DriveBase(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        intakeBase = new IntakeBase(new IntakeIOSpark());
        elevatorBase = new ElevatorBase(new ElevatorIOSpark());
        dispenserBase = new DispenserBase(new DispenserIOSpark());
      }
      case SIM -> {
        driveBase =
            new DriveBase(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        intakeBase = new IntakeBase(new IntakeIOSim());
        elevatorBase = new ElevatorBase(new ElevatorIOSim());
        dispenserBase = new DispenserBase(new DispenserIOSim());
      }
      default -> {
        driveBase =
            new DriveBase(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        intakeBase = new IntakeBase(new IntakeIO() {});
        elevatorBase = new ElevatorBase(new ElevatorIO() {});
        dispenserBase = new DispenserBase(new DispenserIO() {});
      }
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    if (Constants.TUNING_MODE) {
      // Set up Characterization routines
      autoChooser.addOption(
          "Drive Wheel Radius Characterization", driveBase.wheelRadiusCharacterization());
      autoChooser.addOption(
          "Drive Simple FF Characterization", driveBase.feedforwardCharacterization());

      // autoChooser.addOption(
      //     "Elevator Dynamic Forward",
      //     superstructureBase.sysIdDynamic(SysIdRoutine.Direction.kForward));
      // autoChooser.addOption(
      //     "Elevator Dynamic Reverse",
      //     superstructureBase.sysIdDynamic(SysIdRoutine.Direction.kReverse));
      // autoChooser.addOption(
      //     "Elevator Quasi Forward",
      //     superstructureBase.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      // autoChooser.addOption(
      //     "Elevator Quasi Reverse",
      //     superstructureBase.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    }

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    driveBase.setDefaultCommand(
        DriveCommands.joystickDrive(
            driveBase,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                driveBase,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    controller.back().onTrue(elevatorBase.homingSequence());

    controller.povDown().onTrue(Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.STOW)));
    controller
        .povLeft()
        .onTrue(Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.L1_CORAL)));
    controller.povUp().onTrue(Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.L2_CORAL)));
    controller
        .povRight()
        .onTrue(Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.L3_CORAL)));

    // // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(driveBase::stopWithX, driveBase));

    // // Reset gyro to 0° when B button is pressed
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     PoseEstimator.getInstance()
    //                         .resetPose(
    //                             new Pose2d(
    //
    // PoseEstimator.getInstance().getEstimatedPose().getTranslation(),
    //                                 AllianceFlipUtil.apply(new Rotation2d()))),
    //                 driveBase)
    //             .ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
