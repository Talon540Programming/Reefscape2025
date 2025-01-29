package frc.robot.util;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class AlertsUtil {
  private static final double LOW_VOLTAGE_WARNING_THRESHOLD = 11.75;

  private static AlertsUtil instance;

  public static AlertsUtil getInstance() {
    if (instance == null) {
      instance = new AlertsUtil();
    }

    return instance;
  }

  // System Alerts
  private final Alert canErrorAlert =
      new Alert("CAN errors detected, robot may not be controllable.", AlertType.kError);
  private final Debouncer canErrorDebouncer = new Debouncer(0.5);

  private final Alert lowBatteryVoltageAlert =
      new Alert("Battery voltage is too low, change the battery", AlertType.kWarning);
  private final Debouncer batteryVoltageDebouncer = new Debouncer(1.5);

  // Program Alerts
  private final Alert tuningModeAlert = new Alert("Robot in Tuning Mode", AlertType.kInfo);

  private AlertsUtil() {
    if (Constants.TUNING_MODE) {
      tuningModeAlert.set(true);
    }
  }

  public void periodic() {
    // Update System Alerts
    // Check CAN status
    var canStatus = RobotController.getCANStatus();
    canErrorAlert.set(
        canErrorDebouncer.calculate(
            canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0));

    // Update Battery Voltage Alert
    lowBatteryVoltageAlert.set(
        batteryVoltageDebouncer.calculate(
            RobotController.getBatteryVoltage() <= LOW_VOLTAGE_WARNING_THRESHOLD));
  }
}
