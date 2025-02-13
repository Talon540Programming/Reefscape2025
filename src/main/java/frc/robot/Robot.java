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

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.LoggerUtil;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private static final double LOW_VOLTAGE_WARNING_THRESHOLD = 11.75;

  private Command autonomousCommand;
  private final RobotContainer robotContainer;

  // System Alerts
  private final Alert canErrorAlert =
      new Alert("CAN errors detected, robot may not be controllable.", AlertType.kError);
  private final Debouncer canErrorDebouncer = new Debouncer(0.5);

  private final Alert lowBatteryVoltageAlert =
      new Alert("Battery voltage is too low, change the battery", Alert.AlertType.kWarning);
  private final Debouncer batteryVoltageDebouncer = new Debouncer(1.5);

  public Robot() {
    super(Constants.kLoopPeriodSecs);

    LoggerUtil.initializeLoggerMetadata();

    switch (Constants.getMode()) {
      case REAL -> {
        // Running on a real robot, log to a USB stick
        var loggerPath = LoggerUtil.getLogPath();
        if (loggerPath.isPresent()) {
          Logger.addDataReceiver(new WPILOGWriter(loggerPath.get().toString()));
        } else {
          DriverStation.reportWarning("Logging USB Drive Not Found. Disabling File Logging", false);
        }

        Logger.addDataReceiver(new NT4Publisher());
      }
      case SIM -> {
        Logger.addDataReceiver(new NT4Publisher());
      }
      case REPLAY -> {
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
      }
    }

    // Initialize URCL
    Logger.registerURCL(URCL.startExternal());

    // Start AdvantageKit logger
    if (Constants.ENABLE_LOGGING) Logger.start();

    // Configure brownout voltage
    RobotController.setBrownoutVoltage(6.0);

    // Create RobotConatiner
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 99);

    // Run command scheduler
    CommandScheduler.getInstance().run();

    // Check CAN status
    var canStatus = RobotController.getCANStatus();
    canErrorAlert.set(
        canErrorDebouncer.calculate(
            canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0));

    // Update Battery Voltage Alert
    lowBatteryVoltageAlert.set(
        batteryVoltageDebouncer.calculate(
            RobotController.getBatteryVoltage() <= LOW_VOLTAGE_WARNING_THRESHOLD));

    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
