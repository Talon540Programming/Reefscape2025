package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
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
import frc.robot.subsystems.reef_selector.ReefSelectorBase;
import frc.robot.subsystems.vision.*;
import frc.robot.util.DoublePressTracker;
import frc.robot.util.MirrorUtil;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

@ExtensionMethod({DoublePressTracker.class, LEDBase.class})
public class RobotContainer {
  // Subsystems
  private DriveBase driveBase;
  private IntakeBase intakeBase;
  private ElevatorBase elevatorBase;
  private DispenserBase dispenserBase;
  private VisionBase visionBase;
  private ReefSelectorBase reefSelectorBase;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final LoggedNetworkNumber endgameAlert1 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #1", 30.0);
  private final LoggedNetworkNumber endgameAlert2 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #2", 15.0);

  private final LoggedNetworkBoolean disableReefAutoAlign =
      new LoggedNetworkBoolean("/SmartDashboard/ReefAutoAlignDisabled", false);
  private final LoggedNetworkBoolean disableCoralStationAutoAlign =
      new LoggedNetworkBoolean("/SmartDashboard/CoralStationAutoAlignDisabled", false);

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
    }

    // Initialize no-op implementations
    if (driveBase == null) {
      driveBase =
          new DriveBase(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }

    if (intakeBase == null) {
      intakeBase = new IntakeBase(new IntakeIO() {});
    }

    if (elevatorBase == null) {
      elevatorBase = new ElevatorBase(new ElevatorIO() {});
    }

    if (dispenserBase == null) {
      dispenserBase = new DispenserBase(new DispenserIO() {});
    }

    if (visionBase == null) {
      visionBase = new VisionBase(new VisionIO() {}, new VisionIO() {});
    }

    reefSelectorBase = new ReefSelectorBase();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    LoggedDashboardChooser<Boolean> mirror =
        new LoggedDashboardChooser<>("Processor Side?"); // TODO more descriptive name
    mirror.addDefaultOption("Yes", false);
    mirror.addOption("No", true);
    MirrorUtil.setMirror(mirror::get);

    var autoBuilder = new AutoBuilder(driveBase, elevatorBase, dispenserBase, intakeBase);
    autoChooser.addDefaultOption("Noting", Commands.none());
    autoChooser.addOption("Taxi", autoBuilder.taxi());
    // autoChooser.addOption("Deadreckoned L2", autoBuilder.deadreckonedL2());
    // autoChooser.addOption("Deadreckoned L2 Center Start", autoBuilder.deadreckonedL2Center());
    // autoChooser.addOption("Deadreckoned L4", autoBuilder.deadreckonedL4());
    // autoChooser.addOption("Deadreckoned L4 Center Start", autoBuilder.deadreckonedL4Center());

    if (Constants.TUNING_MODE) {
      // Set up Characterization routines
      autoChooser.addOption(
          "Drive Wheel Radius Characterization", driveBase.wheelRadiusCharacterization());
      autoChooser.addOption(
          "Drive Simple FF Characterization", driveBase.feedforwardCharacterization());
    }

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Drive suppliers. Allows both driver and operator to have control over bot.
    DoubleSupplier driverX = () -> -driver.getLeftY() - operator.getLeftY();
    DoubleSupplier driverY = () -> -driver.getLeftX() - operator.getLeftX();
    DoubleSupplier driverOmega = () -> -driver.getRightX() - operator.getRightX();
    BooleanSupplier robotRelative =
        () -> {
          var hid = driver.getHID();
          return hid.getLeftBumperButtonPressed() && hid.getRightBumperButtonPressed();
        };

    // Joystick drive command (driver and operator)
    Supplier<Command> joystickDriveCommandFactory =
        () -> DriveCommands.joystickDrive(driveBase, driverX, driverY, driverOmega, robotRelative);
    driveBase.setDefaultCommand(joystickDriveCommandFactory.get());

    // DRIVER CONTROLLER

    // OPERATOR CONTROLLER

    // // Stow
    // controller
    //     .a()
    //     .debounce(0.25)
    //     .onTrue(Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.STOW)));
    //
    // // Stuck Coral
    // controller
    //     .a()
    //     .doublePress()
    //     .onTrue(Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.STUCK_CORAL)));
    //
    // // L1
    // controller
    //     .povDown()
    //     .onTrue(Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.L1_CORAL)));
    //
    // // L2
    // controller
    //     .povLeft()
    //     .onTrue(Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.L2_CORAL)));
    //
    // // L3
    // controller.povUp().onTrue(Commands.runOnce(() ->
    // elevatorBase.setGoal(ElevatorState.L3_CORAL)));
    //
    // // L4
    // controller
    //     .povRight()
    //     .onTrue(Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.L4_CORAL)));
    //
    // // Intake
    // controller.x().toggleOnTrue(IntakeCommands.intake(elevatorBase, intakeBase, dispenserBase));
    //
    // // Dispense
    // controller
    //     .rightTrigger()
    //     .onTrue(
    //         Commands.either(
    //             dispenserBase.eject(elevatorBase::getGoal),
    //             IntakeCommands.reserialize(elevatorBase, intakeBase, dispenserBase),
    //             controller.leftTrigger().negate().debounce(0.25)));
    //
    // // // Auto Align
    // // controller
    // //     .leftBumper()
    // //     .whileTrue(AutoScoreCommands.autoAlign(driveBase, AutoScoreCommands.ReefSide.Left));
    // // controller
    // //     .rightBumper()
    // //     .whileTrue(AutoScoreCommands.autoAlign(driveBase, AutoScoreCommands.ReefSide.Right));
    //
    // controller
    //     .y()
    //     .whileTrue(
    //         Commands.startEnd(
    //                 () -> LEDBase.getInstance().humanPlayerAlert = true,
    //                 () -> LEDBase.getInstance().humanPlayerAlert = false)
    //             .withName("Strobe LEDBase at HP"));
    //
    // // Reset Gyro
    // controller
    //     .start()
    //     .and(controller.back())
    //     .debounce(0.5)
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

    // Endgame
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get()))
        .onTrue(
            Commands.startEnd(
                    () -> {
                      driver.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
                      operator.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
                    },
                    () -> {
                      driver.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
                      operator.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
                    })
                .withTimeout(0.5)
                .setLEDState((visionBase, state) -> visionBase.endgameAlert = state)
                .withName("Controller Endgame Alert 1"));

    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            Commands.startEnd(
                    () -> {
                      driver.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
                      operator.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
                    },
                    () -> {
                      driver.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
                      operator.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
                    })
                .withTimeout(0.2)
                .andThen(Commands.waitSeconds(0.1))
                .repeatedly()
                .withTimeout(0.9)
                .setLEDState((visionBase, state) -> visionBase.endgameAlert = state)
                .withName("Controller Endgame Alert 2")); // Rumble three times
  }

  // Update dashboard data
  public void updateDashboardOutputs() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  public void updateAlerts() {}

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
