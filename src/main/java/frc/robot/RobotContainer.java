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
import frc.robot.constants.Constants;
import frc.robot.oi.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Elevator elevator;

  // Controller
  //   private final CommandXboxController controller = new CommandXboxController(0);
  private final ControlsInterface controlsInterface = new SingleXbox(0);
  // private final ControlsInterface controlsInterface = new SimKeyboard(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getRobotMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        elevator = new Elevator(new ElevatorIOSpark());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("AutoChooser");

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption(
        "Elevator SysId (Quasistatic Forward)",
        elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Elevator SysId (Quasistatic Reverse)",
        elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Elevator SysId (Dynamic Forward)", elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Elevator SysId (Dynamic Reverse)", elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> controlsInterface.getDriveX(),
            () -> controlsInterface.getDriveY(),
            () -> controlsInterface.getDriveTheta()));

    // Lock to 0° when A button is held
    controlsInterface
        .robotRelativeOverride()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> controlsInterface.getDriveY(),
                () -> controlsInterface.getDriveX(),
                () -> new Rotation2d()));

    // Set to L1 deposit state
    controlsInterface
        .depositL1()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(
                    () -> {
                      elevator.setGoal(() -> ElevatorConstants.L1_STATE);
                      ;
                    },
                    elevator),
                Commands.waitUntil(() -> elevator.isAtGoal())));

    // // Set to L2 left deposit state
    // controlsInterface
    //     .depositL2left()
    //     .onTrue(
    //         Commands.sequence(
    //             Commands.runOnce(
    //                 () -> {
    //                   elevator.setSetpoint(ElevatorState.L2_STATE);
    //                 },
    //                 elevator),
    //             Commands.waitUntil(() -> elevator.atSetpoint())));

    // // Set to L2 right deposit state
    // controlsInterface
    //     .depositL2right()
    //     .onTrue(
    //         Commands.sequence(
    //             Commands.runOnce(
    //                 () -> {
    //                   elevator.setSetpoint(ElevatorState.L2_STATE);
    //                 },
    //                 elevator),
    //             Commands.waitUntil(() -> elevator.atSetpoint())));

    // // Set to L1 deposit state
    // controlsInterface
    //     .depositL3left()
    //     .onTrue(
    //         Commands.sequence(
    //             Commands.runOnce(
    //                 () -> {
    //                   elevator.setSetpoint(ElevatorState.L3_STATE);
    //                 },
    //                 elevator),
    //             Commands.waitUntil(() -> elevator.atSetpoint())));

    // // Set to L1 deposit state
    // controlsInterface
    //     .depositL3right()
    //     .onTrue(
    //         Commands.sequence(
    //             Commands.runOnce(
    //                 () -> {
    //                   elevator.setSetpoint(ElevatorState.L3_STATE);
    //                 },
    //                 elevator),
    //             Commands.waitUntil(() -> elevator.atSetpoint())));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
