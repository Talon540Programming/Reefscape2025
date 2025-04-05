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
import frc.robot.subsystems.vision.*;
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class RobotContainer {
  // Subsystems
  private final DriveBase driveBase;
  private final IntakeBase intakeBase;
  private final ElevatorBase elevatorBase;
  private final DispenserBase dispenserBase;
  //   private final VisionBase visionBase;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final LoggedNetworkNumber endgameAlert1 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #1", 30.0);
  private final LoggedNetworkNumber endgameAlert2 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #2", 15.0);

  private boolean slowModeEnabled = false;
  private boolean robotRelativeDisabled = false;
  private boolean robotRelativeEnabled = false;

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
        // visionBase =
        //     new VisionBase(
        //         new VisionIOPhotonCamera(0)
        //         // , new VisionIOPhotonCamera(1)
        //         );
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
        // visionBase =
        //     new VisionBase(
        //         new VisionIOSim(0)
        //         // , new VisionIOSim(1)
        //         );
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
        // visionBase =
        //     new VisionBase(
        //         new VisionIO() {}
        //         // , new VisionIO() {}
        //         );
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

    // Default command, normal field-relative drive
    driveBase.setDefaultCommand(
        DriveCommands.joystickDrive(
            driveBase,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX(),
            () -> slowModeEnabled,
            () -> robotRelativeEnabled));

    // Stow
    controller
<<<<<<< Updated upstream
        .povDown()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      slowModeEnabled = false;
                      robotRelativeEnabled = false;
                    })
                .andThen(Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.STOW))));
    // L1
    // controller
    //     .povLeft()
    //     .onTrue(Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.L1_CORAL)));
    controller
        .povLeft()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      slowModeEnabled = true;
                      robotRelativeEnabled = true;
                    })
                .andThen(Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.L2_CORAL))));
=======
        .leftTrigger()
        .whileTrue(
            Commands.either(
                    joystickDriveCommandFactory.get(),
                    new DriveToStation(driveBase, driverX, driverY, driverOmega, false),
                    disableCoralStationAutoAlign::get)
                .alongWith(IntakeCommands.intake(elevatorBase, intakeBase, dispenserBase))
                .withName("Coral Station Intake"));
>>>>>>> Stashed changes

    controller
        .povUp()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      slowModeEnabled = true;
                      robotRelativeEnabled = true;
                    })
                .andThen(Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.L3_CORAL))));

    controller
        .povRight()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      slowModeEnabled = true;
                      robotRelativeEnabled = true;
                    })
                .andThen(Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.L4_CORAL))));

<<<<<<< Updated upstream
    // Intake
    controller.x().toggleOnTrue(IntakeCommands.intake(elevatorBase, intakeBase, dispenserBase));
=======
    // Coral intake
    controller
        .x()
        .toggleOnTrue(
            IntakeCommands.intake(elevatorBase, intakeBase, dispenserBase)
                .withName("Operator Coral Intake"));
    // operator
    //     .rightBumper()
    //     .toggleOnTrue(
    //         IntakeCommands.intake(elevatorBase, intakeBase, dispenserBase)
    //             .withName("Operator Coral Intake"));

    // Home elevator
    controller.b().doublePress().onTrue(elevatorBase.homingSequence().withName("Home Elevator"));

    // operator
    //     .y()
    //     .doublePress()
    //     .onTrue(elevatorBase.homingSequence().withName("Operator Home Elevator"));

    BiConsumer<Trigger, FieldConstants.ReefLevel> bindOperatorCoralScore =
        (faceButton, level) -> {
          faceButton.whileTrueRepeatedly(
              elevatorBase
                  .runGoal(() -> Preset.fromLevel(level))
                  .withName("Operator Score on " + level));
          // faceButton
          //     // .and(operator.rightTrigger())
          //     .and(driver.rightTrigger())
          //     .whileTrueRepeatedly(
          //         elevatorBase
          //             .runGoal(() -> Preset.fromLevel(level))
          //             .until(elevatorBase::atGoal)
          //             .andThen(
          //                 dispenserBase.runDispenser(
          //                     () -> dispenserBase.getDispenserVoltageFromLevel(level)))
          //             .withName("Operator Score & Eject On " + level));
        };
>>>>>>> Stashed changes

    // Dispense
    controller
        .rightTrigger()
        .onTrue(
            Commands.either(
                dispenserBase.eject(elevatorBase::getGoal),
                // .andThen(Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.STOW))),
                IntakeCommands.reserialize(elevatorBase, intakeBase, dispenserBase),
                controller.leftTrigger().negate().debounce(0.25)));

<<<<<<< Updated upstream
    // Home Elevator
    controller
        .back()
        .and(controller.start().negate())
        .debounce(0.5)
        .onTrue(elevatorBase.homingSequence());
=======
    // bindOperatorCoralScore.accept(operator.povDown(), ReefLevel.L1);
    // bindOperatorCoralScore.accept(operator.povLeft(), ReefLevel.L2);
    // bindOperatorCoralScore.accept(operator.povUp(), ReefLevel.L3);
    // bindOperatorCoralScore.accept(operator.povRight(), ReefLevel.L4);
    bindOperatorCoralScore.accept(controller.povDown(), ReefLevel.L1);
    bindOperatorCoralScore.accept(controller.povLeft(), ReefLevel.L2);
    bindOperatorCoralScore.accept(controller.povUp(), ReefLevel.L3);
    bindOperatorCoralScore.accept(controller.povRight(), ReefLevel.L4);
>>>>>>> Stashed changes

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

    new Trigger(
            () ->
                (elevatorBase.getGoal() != ElevatorState.STOW
                    || elevatorBase.getGoal() != ElevatorState.INTAKE
                    || !robotRelativeDisabled))
        .onTrue(
            Commands.runOnce(
                () -> {
                  slowModeEnabled = true;
                  robotRelativeEnabled = true;
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  slowModeEnabled = false;
                  robotRelativeEnabled = false;
                }));

    controller
        .a()
        .onTrue(
            Commands.run(
                () -> {
                  robotRelativeDisabled = true;
                }));

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
