// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOPhotonCamera;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.PoseEstimator;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  //   private final ControlsInterface controlsInterface = new SingleXbox(0);
  //   private final ControlsInterface controlsInterface = new SimKeyboard(0);
  private final boolean slowModeEnabled = false;

  // Load PoseEstimator class
  private final PoseEstimator poseEstimator = PoseEstimator.getInstance();

  // Subsystems
  private final DriveBase driveBase;
  private final IntakeBase intakeBase;
  private final ElevatorBase elevatorBase;
  private final DispenserBase dispenserBase;
  private final Vision vision;

  // Controller
  //   private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final LoggedNetworkNumber endgameAlert1 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #1", 30.0);
  private final LoggedNetworkNumber endgameAlert2 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #2", 15.0);

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

        // vision =
        //     new Vision(
        //         VisionConstants.cameras.stream()
        //             .map(
        //                 v ->
        //                     new VisionIOPhotonCamera(
        //                         v.cameraName(), v.robotToCamera(), v.cameraBias()))
        //             .toArray(VisionIOPhotonCamera[]::new));

        // Sim robot, instantiate physics sim IO implementations
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
        // vision =
        //     new Vision(
        //         VisionConstants.cameras.stream()
        //             .map(
        //                 v ->
        //                     new VisionIOSim(
        //                         v.cameraName(),
        //                         v.robotToCamera(),
        //                         v.cameraBias(),
        //
        // Path.of("camera_calibrations/mrcalibration_testcam@1280x800.json")))
        //             .toArray(VisionIOSim[]::new));

        // Replayed robot, disable IO implementations
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
        // vision = new Vision(new VisionIO[] {});
        intakeBase = new IntakeBase(new IntakeIO() {});
        elevatorBase = new ElevatorBase(new ElevatorIO() {});
        dispenserBase = new DispenserBase(new DispenserIO() {});
      }
    }

    vision =
        new Vision(
            VisionConstants.cameras.stream()
                .map(
                    v ->
                        new VisionIOPhotonCamera(v.cameraName(), v.robotToCamera(), v.cameraBias()))
                .toArray(VisionIOPhotonCamera[]::new));

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

      autoChooser.addOption(
          "Drive Dynamic Forward", driveBase.sysIdDynamic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption(
          "Drive Dynamic Reverse", driveBase.sysIdDynamic(SysIdRoutine.Direction.kReverse));
      autoChooser.addOption(
          "Drive Quasi Forward", driveBase.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption(
          "Drive Quasi Reverse", driveBase.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    }

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    driveBase.setDefaultCommand(
        DriveCommands.joystickDrive(
            driveBase,
            () ->
                controller.y().getAsBoolean()
                    ? -controller.getLeftY() * 0.7
                    : -controller.getLeftY(),
            () ->
                controller.y().getAsBoolean()
                    ? -controller.getLeftX() * 0.7
                    : -controller.getLeftX(),
            () ->
                controller.y().getAsBoolean()
                    ? -controller.getRightX() * 0.7
                    : -controller.getRightX(),
            controller.leftStick()));

    // TODO Lock to Feeder Station when held
    // controller
    //     .y()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             driveBase,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> Rotation2d.kZero,
    //             controller.leftStick()));

    // Stow
    controller.povDown().onTrue(Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.STOW)));
    // Intake
    controller.x().onTrue(IntakeCommands.intake(elevatorBase, intakeBase, dispenserBase));
    // L1
    controller
        .povLeft()
        .onTrue(Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.L1_CORAL)));
    // L2
    controller.povUp().onTrue(Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.L2_CORAL)));
    // L3
    controller
        .povRight()
        .onTrue(Commands.runOnce(() -> elevatorBase.setGoal(ElevatorState.L3_CORAL)));
    // Dispense
    controller.rightTrigger().onTrue(dispenserBase.eject());

    // Home Elevator
    controller.back().debounce(0.5).onTrue(elevatorBase.homingSequence());

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
