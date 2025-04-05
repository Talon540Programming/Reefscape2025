package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.commands.*;
import frc.robot.commands.AutoBuilder;
import frc.robot.subsystems.dispenser.DispenserBase;
import frc.robot.subsystems.dispenser.DispenserIO;
import frc.robot.subsystems.dispenser.DispenserIOSim;
import frc.robot.subsystems.dispenser.DispenserIOSpark;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.elevator.ElevatorPose.Preset;
import frc.robot.subsystems.intake.IntakeBase;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.leds.LEDBase;
import frc.robot.subsystems.vision.*;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.DoublePressTracker;
import frc.robot.util.MirrorUtil;
import frc.robot.util.TriggerUtil;
import java.util.*;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

@ExtensionMethod({DoublePressTracker.class, TriggerUtil.class, LEDBase.class})
public class RobotContainer {
  // Subsystems
  private DriveBase driveBase;
  private IntakeBase intakeBase;
  private ElevatorBase elevatorBase;
  private DispenserBase dispenserBase;
  private VisionBase visionBase;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  private final Alert controllerDisconnected =
      new Alert("Main controller disconnected (port 0).", Alert.AlertType.kWarning);

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

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    LoggedDashboardChooser<Boolean> mirror =
        new LoggedDashboardChooser<>("Starting on Processor Side?");
    mirror.addDefaultOption("Yes", false);
    mirror.addOption("No", true);
    MirrorUtil.setMirror(mirror::get);

    var autoBuilder = new AutoBuilder(driveBase, elevatorBase, dispenserBase, intakeBase);
    autoChooser.addDefaultOption("Noting", Commands.none());
    autoChooser.addOption("Chungus", autoBuilder.chungusAuto(false));
    autoChooser.addOption("DeadreckonedChungus", autoBuilder.chungusAuto(true));
    autoChooser.addOption("SuperSafeSkibidiL4", autoBuilder.superSafeSkibidiL4());
    autoChooser.addOption("SuperSafeSideStartSkibidiL4", autoBuilder.superSafeSideStartSkibidiL4());
    autoChooser.addOption("Taxi", autoBuilder.taxi());
    autoChooser.addOption("PracticeFieldAuto", autoBuilder.practiceFieldAuto());
    autoChooser.addOption("PracticeFieldAutoMulti", autoBuilder.practiceFieldMultiAuto());

    if (Constants.TUNING_MODE) {
      // Set up Characterization routines
      autoChooser.addOption(
          "Drive Wheel Radius Characterization", driveBase.wheelRadiusCharacterization());
      autoChooser.addOption(
          "Drive Simple FF Characterization", driveBase.feedforwardCharacterization());
    }

    registerStateTriggers();
    configureButtonBindings();
  }

  private void registerStateTriggers() {
    // Put the elevator down if the robot is starting to tip
    double tipThresholdDeg = 10.0;
    new Trigger(
            () ->
                Math.abs(RobotState.getInstance().getPitch().getDegrees()) > tipThresholdDeg
                    || Math.abs(RobotState.getInstance().getRoll().getDegrees()) > tipThresholdDeg)
        .setLEDState((visionBase, state) -> visionBase.robotTipping = state)
        .whileTrueRepeatedly(
            elevatorBase.runGoal(Preset.STOW).withName("Automatic Tip Prevention"));
  }

  private void configureButtonBindings() {
    // Drive suppliers. Allows both driver and operator to have control over bot.
    DoubleSupplier driverX = () -> -controller.getLeftY();
    DoubleSupplier driverY = () -> -controller.getLeftX();
    DoubleSupplier driverOmega = () -> -controller.getRightX();
    // TODO
    // BooleanSupplier robotRelative =
    //     () -> controller.leftBumper().and(controller.rightBumper()).getAsBoolean();
    BooleanSupplier robotRelative = () -> false;

    // Joystick drive command (driver and operator)
    Supplier<Command> joystickDriveCommandFactory =
        () -> DriveCommands.joystickDrive(driveBase, driverX, driverY, driverOmega, robotRelative);
    driveBase.setDefaultCommand(joystickDriveCommandFactory.get());

    BiConsumer<Trigger, Boolean> bindAutoAlign =
        (faceButton, isLeftSide) -> {
          faceButton.whileTrue(
              AutoScoreCommands.autoAlign(
                  driveBase,
                  elevatorBase,
                  () -> {
                    Pose2d robot = RobotState.getInstance().getEstimatedPose();
                    Map<Integer, Pose2d> centerFaces = new HashMap<>();
                    for (int i = 0; i < 6; i++) {
                      centerFaces.put(
                          i, AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[i]));
                    }
                    int closesFaceIdx =
                        Collections.min(
                                centerFaces.entrySet(),
                                Comparator.comparing(
                                        (Map.Entry<Integer, Pose2d> other) ->
                                            robot
                                                .getTranslation()
                                                .getDistance(other.getValue().getTranslation()))
                                    .thenComparing(
                                        (var other) ->
                                            Math.abs(
                                                robot
                                                    .getRotation()
                                                    .minus(other.getValue().getRotation())
                                                    .getRadians())))
                            .getKey();

                    return Optional.of(
                        new FieldConstants.CoralObjective(
                            closesFaceIdx * 2 + (isLeftSide ? 1 : 0), ReefLevel.L4));
                  },
                  driverX,
                  driverY,
                  driverOmega));
        };

    bindAutoAlign.accept(controller.leftBumper(), true);
    bindAutoAlign.accept(controller.rightBumper(), false);

    // Coral intake
    controller
        .leftTrigger()
        .whileTrue(
            Commands.either(
                    joystickDriveCommandFactory.get(),
                    new DriveToStation(driveBase, driverX, driverY, driverOmega, false),
                    disableCoralStationAutoAlign::get)
                .alongWith(IntakeCommands.intake(elevatorBase, intakeBase, dispenserBase))
                .withName("Coral Station Intake"));

    controller
        .y()
        .whileTrue(
            Commands.startEnd(
                    () -> LEDBase.getInstance().humanPlayerAlert = true,
                    () -> LEDBase.getInstance().humanPlayerAlert = false)
                .withName("Strobe LEDBase at HP"));

    // Stow
    controller
        .a()
        .debounce(0.1)
        .onTrue(elevatorBase.runGoal(Preset.STOW).withName("Stow Elevator"));
    // Stuck Coral (Chunus Solutions)
    controller
        .a()
        .doublePress()
        .onTrue(elevatorBase.runGoal(Preset.STUCK_CORAL).withName("Stow Elevator Coral Stuck"));

    // Coral intake
    controller
        .x()
        .toggleOnTrue(
            IntakeCommands.intake(elevatorBase, intakeBase, dispenserBase)
                .withName("Operator Coral Intake"));

    // Home elevator
    controller.b().doublePress().onTrue(elevatorBase.homingSequence().withName("Home Elevator"));

    BiConsumer<Trigger, FieldConstants.ReefLevel> bindOperatorCoralScore =
        (faceButton, level) -> {
          faceButton.whileTrueRepeatedly(
              elevatorBase
                  .runGoal(() -> ElevatorBase.getScoringState(level))
                  .withName("Operator Score on " + level));
        };

    bindOperatorCoralScore.accept(controller.povDown(), ReefLevel.L1);
    bindOperatorCoralScore.accept(controller.povLeft(), ReefLevel.L2);
    bindOperatorCoralScore.accept(controller.povUp(), ReefLevel.L3);
    bindOperatorCoralScore.accept(controller.povRight(), ReefLevel.L4);

    // var obj = new FieldConstants.CoralObjective(3, ReefLevel.L4);
    // controller
    //     .b()
    //     .whileTrue(
    //         AutoScoreCommands.autoScore(
    //             driveBase,
    //             elevatorBase,
    //             dispenserBase,
    //             intakeBase,
    //             obj::reefLevel,
    //             () -> Optional.of(obj)));

    controller
        .rightTrigger()
        .onTrue(
            dispenserBase
                .runDispenser(
                    () ->
                        dispenserBase.getDispenserVoltageFromLevel(
                            switch (elevatorBase.getGoal()) {
                              case L1_CORAL -> ReefLevel.L1;
                              case L2_CORAL -> ReefLevel.L2;
                              case L3_CORAL -> ReefLevel.L3;
                              case L4_CORAL -> ReefLevel.L4;
                              default -> ReefLevel.L2;
                            }))
                .withTimeout(0.75));

    // ***** MISCELlANEOUS *****
    // Reset gyro
    controller
        .start()
        .and(controller.back())
        .debounce(0.5)
        .onTrue(
            Commands.runOnce(
                    () ->
                        RobotState.getInstance()
                            .resetPose(
                                new Pose2d(
                                    RobotState.getInstance().getEstimatedPose().getTranslation(),
                                    AllianceFlipUtil.apply(Rotation2d.kZero))))
                .withName("Reset Gyro")
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
                .setLEDState((visionBase, state) -> visionBase.endgameAlert = state)
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
                .setLEDState((visionBase, state) -> visionBase.endgameAlert = state)
                .withName("Controller Endgame Alert 2")); // Rumble three times
  }

  // Update dashboard data
  public void updateDashboardOutputs() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  public void updateAlerts() {
    // Controller disconnected alerts
    controllerDisconnected.set(!DriverStation.isJoystickConnected(controller.getHID().getPort()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
