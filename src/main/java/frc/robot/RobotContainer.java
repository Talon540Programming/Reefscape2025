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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.oi.ControlsInterface;
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
import frc.robot.util.PoseEstimator;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems

  // Controller
  //   private final CommandXboxController controller = new CommandXboxController(0);
  //   private final ControlsInterface controlsInterface = new SingleXbox(0);
  private final ControlsInterface controlsInterface = new SimKeyboard(0);
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

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
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
        elevator = new Elevator(new ElevatorIOSpark());

        // vision =
        //     new Vision(
        //         VisionConstants.cameras.stream()
        //             .map(
        //                 v ->
        //                     new VisionIOPhotonCamera(
        //                         v.cameraName(), v.robotToCamera(), v.cameraBias()))
        //             .toArray(VisionIOPhotonCamera[]::new));

        break;

      case SIM:
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
        elevator = new Elevator(new ElevatorIOSim());
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
        break;

      default:
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
        elevator = new Elevator(new ElevatorIO() {});
        // vision = new Vision(new VisionIO[] {});
        break;
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
                !controlsInterface.slowMode().getAsBoolean()
                    ? controlsInterface.getDriveX()
                    : controlsInterface.getDriveX() * 0.5,
            () ->
                !controlsInterface.slowMode().getAsBoolean()
                    ? controlsInterface.getDriveY()
                    : controlsInterface.getDriveY() * 0.5,
            () ->
                !controlsInterface.slowMode().getAsBoolean()
                    ? controlsInterface.getDriveTheta()
                    : controlsInterface.getDriveTheta() * 0.5));

    // Lower elevator and run rollers and intake belts to intake coral
    // controlsInterface.intake().onTrue(

    // )

    // Lock to 0° when A button is held
    controlsInterface
        .robotRelativeOverride()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                driveBase,
                () -> -controlsInterface.getDriveY(),
                () -> -controlsInterface.getDriveX(),
                () -> Rotation2d.kZero));

    controlsInterface.stopWithX().onTrue(Commands.runOnce(driveBase::stopWithX, driveBase));

    controlsInterface
        .resetGyro()
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


    controlsInterface
        .enableReefAutoAlignment()
        .whileTrue(
            Commands.runOnce(
                    () -> {
                      vision.setPipelineIndex(1);
                    },
                    vision)
                .andThen(
                    new DriveToReef(
                        driveBase,
                        () ->
                            vision
                                .getNearestReefPose()
                                .plus(FieldConstants.Reef.robotFromBranchOffset),
                        () ->
                            PoseEstimator.getInstance()
                                .getReefPose(
                                    vision.getReefAprilTagId(), vision.getNearestReefPose())))
                .until(
                    () ->
                        EqualsUtil.GeomExtensions.epsilonEquals(
                            PoseEstimator.getInstance().getEstimatedPose(),
                            vision
                                .getNearestReefPose()
                                .plus(FieldConstants.Reef.robotFromBranchOffset)))
                .finallyDo(
                    () -> {
                      vision.setPipelineIndex(0);
                    }));
    
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
