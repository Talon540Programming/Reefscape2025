package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
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
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

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

  private final LoggedNetworkNumber endgameAlert1 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #1", 30.0);
  private final LoggedNetworkNumber endgameAlert2 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #2", 15.0);

  private boolean slowModeEnabled;

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
      autoChooser.addOption(
          "Elevator Dynamic Forward",
          elevatorBase.sysIdDynamic(SysIdRoutine.Direction.kForward, 0.5));
      autoChooser.addOption(
          "Elevator Dynamic Reverse",
          elevatorBase.sysIdDynamic(SysIdRoutine.Direction.kReverse, 0.25));
      autoChooser.addOption(
          "Elevator Quasi Forward",
          elevatorBase.sysIdQuasistatic(SysIdRoutine.Direction.kForward, 25));
      autoChooser.addOption(
          "Elevator Quasi Reverse",
          elevatorBase.sysIdQuasistatic(SysIdRoutine.Direction.kReverse, 3));
    }

    autoChooser.addDefaultOption("Noting", Commands.none());
    autoChooser.addOption(
        "Taxi",
        Commands.runEnd(
                () -> driveBase.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0)),
                driveBase::stop,
                driveBase)
            .withTimeout(2.0)
            .beforeStarting(
                Commands.runOnce(
                    () ->
                        PoseEstimator.getInstance()
                            .resetPose(
                                new Pose2d(
                                    PoseEstimator.getInstance().getEstimatedPose().getTranslation(),
                                    AllianceFlipUtil.apply(Rotation2d.kPi))),
                    driveBase)));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Make slow mode toggleable
    controller.y().toggleOnTrue(Commands.runOnce(() -> slowModeEnabled = !slowModeEnabled));

    // Default command, normal field-relative drive
    driveBase.setDefaultCommand(
        DriveCommands.joystickDrive(
            driveBase,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX(),
            () -> slowModeEnabled,
            () -> controller.leftBumper().and(controller.rightBumper()).getAsBoolean()));

    // Stow
    controller.povDown().onTrue(Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.STOW)));
    // L1
    controller
        .povLeft()
        .onTrue(Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.L1_CORAL)));
    // L2
    controller
        .povUp()
        .onTrue(
            Commands.either(
                Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.L2_CORAL)),
                Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.L2_ALGAE_REMOVAL)),
                controller.b().negate().debounce(0.25)));

    // L3
    controller
        .povRight()
        .onTrue(
            Commands.either(
                Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.L3_CORAL)),
                Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.L3_ALGAE_REMOVAL)),
                controller.b().negate().debounce(0.25)));

    // Intake
    controller.x().toggleOnTrue(IntakeCommands.intake(elevatorBase, intakeBase, dispenserBase));

    controller
        .rightTrigger()
        .onTrue(
            Commands.either(
                dispenserBase
                    .eject(elevatorBase::getGoal)
                    .andThen(Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.STOW))),
                IntakeCommands.reserialize(elevatorBase, intakeBase, dispenserBase),
                controller.leftTrigger().negate().debounce(0.25)));

    // Home Elevator
    controller
        .back()
        .and(controller.start().negate())
        .debounce(0.5)
        .onTrue(elevatorBase.homingSequence());

    // Auto Align (Left or Right)
    // TODO

    // Human Player Alert (Strobe LEDs)
    // TODO

    // Reset Gyro
    controller
        .start()
        .and(controller.back())
        .debounce(0.5)
        .onTrue(
            Commands.runOnce(
                    () ->
                        PoseEstimator.getInstance()
                            .resetPose(
                                new Pose2d(
                                    PoseEstimator.getInstance().getEstimatedPose().getTranslation(),
                                    AllianceFlipUtil.apply(new Rotation2d()))),
                    driveBase)
                .ignoringDisable(true));

    // Endgame
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get()))
        .onTrue(
            Commands.startEnd(
                    () -> controller.setRumble(GenericHID.RumbleType.kBothRumble, 1.0),
                    () -> controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0))
                .withTimeout(0.5));

    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            Commands.startEnd(
                    () -> controller.setRumble(GenericHID.RumbleType.kBothRumble, 1.0),
                    () -> controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0))
                .withTimeout(0.5));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
