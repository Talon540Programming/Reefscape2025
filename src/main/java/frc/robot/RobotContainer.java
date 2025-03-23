package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoScoreCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.auto.AutoBuilder;
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
import frc.robot.subsystems.leds.LEDBase;
import frc.robot.subsystems.vision.*;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.DoublePressTracker;
import java.util.Arrays;
import java.util.List;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

@ExtensionMethod({DoublePressTracker.class})
public class RobotContainer {
  // Subsystems
  private final DriveBase driveBase;
  private final IntakeBase intakeBase;
  private final ElevatorBase elevatorBase;
  private final DispenserBase dispenserBase;
  private final VisionBase visionBase;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final LoggedNetworkNumber endgameAlert1 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #1", 30.0);
  private final LoggedNetworkNumber endgameAlert2 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #2", 15.0);

  // @AutoLogOutput(key = "AutoScore/Pending")
  // private boolean autoScorePending = false;

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
        visionBase = new VisionBase(new VisionIOPhotonCamera(0), new VisionIOPhotonCamera(1));
        // visionBase = new VisionBase(new VisionIO() {}, new VisionIO() {});
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
        visionBase = new VisionBase(new VisionIOSim(0), new VisionIOSim(1));
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
        visionBase = new VisionBase(new VisionIO() {}, new VisionIO() {});
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
    }

    var autoBuilder = new AutoBuilder(driveBase, elevatorBase, dispenserBase, intakeBase);

    autoChooser.addDefaultOption("Noting", Commands.none());
    autoChooser.addOption("Taxi", autoBuilder.taxi());
    autoChooser.addOption("Deadreckoned L2", autoBuilder.deadreckonedL2());
    autoChooser.addOption("Deadreckoned L2 Center Start", autoBuilder.deadreckonedL2Center());
    autoChooser.addOption("Deadreckoned L4", autoBuilder.deadreckonedL4());
    autoChooser.addOption("Deadreckoned L4 Center Start", autoBuilder.deadreckonedL4Center());

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    driveBase.setDefaultCommand(
        DriveCommands.joystickDrive(
            driveBase,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX(),
            () ->
                !List.of(ElevatorState.INTAKE, ElevatorState.STOW, ElevatorState.START)
                    .contains(elevatorBase.getGoal()),
            () -> controller.leftBumper().and(controller.rightBumper()).getAsBoolean()));

    // Driver Controls
    // Stow
    controller.a().onTrue(Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.STOW)));

    // L1
    controller
        .povDown()
        .onTrue(Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.L1_CORAL)));

    // L2
    controller
        .povLeft()
        .onTrue(Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.L2_CORAL)));

    // L3
    controller.povUp().onTrue(Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.L3_CORAL)));

    // L4
    controller
        .povRight()
        .onTrue(Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.L4_CORAL)));

    // Intake
    controller.x().toggleOnTrue(IntakeCommands.intake(elevatorBase, intakeBase, dispenserBase));

    // Dispense
    controller
        .rightTrigger()
        .onTrue(
            Commands.either(
                dispenserBase.eject(elevatorBase::getGoal),
                // .andThen(Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.STOW))),
                IntakeCommands.reserialize(elevatorBase, intakeBase, dispenserBase),
                controller.leftTrigger().negate().debounce(0.25)));

    // // // Auto Align

    controller
        .leftBumper()
        .whileTrue(
            AutoScoreCommands.autoAlign(
                driveBase,
                () ->
                    PoseEstimator.getInstance()
                        .getEstimatedPose()
                        .nearest(Arrays.asList(FieldConstants.Reef.centerFaces)),
                () -> 0));
    controller
        .rightBumper()
        .whileTrue(
            AutoScoreCommands.autoAlign(
                driveBase,
                () ->
                    PoseEstimator.getInstance()
                        .getEstimatedPose()
                        .nearest(Arrays.asList(FieldConstants.Reef.centerFaces)),
                () -> 1));

    // Human Player Alert
    controller
        .y()
        .whileTrue(
            Commands.startEnd(
                    () -> LEDBase.getInstance().humanPlayerAlert = true,
                    () -> LEDBase.getInstance().humanPlayerAlert = false)
                .withName("Strobe LEDs at HP"));

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
                .withTimeout(0.5)
                .beforeStarting(() -> LEDBase.getInstance().endgameAlert = true)
                .finallyDo(() -> LEDBase.getInstance().endgameAlert = false)
                .withName("Controller Endgame Alert 1"));

    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            Commands.startEnd(
                    () -> controller.setRumble(GenericHID.RumbleType.kBothRumble, 1.0),
                    () -> controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0))
                .withTimeout(0.2)
                .andThen(Commands.waitSeconds(0.1))
                .repeatedly()
                .withTimeout(0.9)
                .beforeStarting(() -> LEDBase.getInstance().endgameAlert = true)
                .finallyDo(() -> LEDBase.getInstance().endgameAlert = false)
                .withName("Controller Endgame Alert 2")); // Rumble three times
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
