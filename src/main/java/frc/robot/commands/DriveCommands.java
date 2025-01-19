package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(() -> {}, drive);
  }

  // /**
  //  * Field relative drive command using joystick for linear control and PID for angular control.
  //  * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
  //  * absolute rotation with a joystick.
  //  */
  // public static Command joystickDriveAtAngle() {}

  /** Measures the velocity feedforward constants for the drive motors. */
  public static Command feedforwardCharacterization() {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence();
  }

  // /** Measures the robot's wheel radius by spinning in a circle. */
  // public static Command wheelRadiusCharacterization() {}
}
