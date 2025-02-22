package frc.robot.util;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

// TODO LED States:
// holding coral
// holding algae
// holding both coral and algae
// following trajectory or go to pose or teleop drive
// auto stage

// TODO a hardware alerts class that can be chained together and be built based on error priority

// TODO LED States:
// holding coral
// holding algae
// holding both coral and algae
// following trajectory or go to pose or teleop drive
// auto stage

public class AlertsUtil {
  private static final int numLEDs = 120; // TODO
  private static final double LOW_VOLTAGE_WARNING_THRESHOLD = 11.75;
  // private static final double LED_DISABLE_VOLTAGE_THRESHOLD = 10.0;

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

  // private final Alert ledsDisabledAlert = new Alert("LEDs Disabled by Override",
  // AlertType.kInfo);
  // private final Alert ledLowVoltageDisabledAlert = new Alert("LEDs disabled due to low battery
  // voltage", AlertType.kWarning);
  // private final Debouncer ledVoltageDebouncer = new Debouncer(1.5);

  // private final Alert holdingCoral = new Alert("Holding Coral", AlertType.kInfo);
  // private final Alert holdingAlgae = new Alert("Holding Algae", AlertType.kInfo);

  // // Hardware Alerts
  // private AddressableLED leds;
  // private AddressableLEDBuffer ledBuffer;

  private AlertsUtil() {
    if (Constants.TUNING_MODE) {
      tuningModeAlert.set(true);
    }

    // if(Constants.ENABLE_LEDs) {
    //   leds = new AddressableLED(0); // TODO
    //   ledBuffer = new AddressableLEDBuffer(numLEDs);
    //
    //   leds.setLength(ledBuffer.getLength());
    //   leds.start();
    // } else {
    //   ledsDisabledAlert.set(true);
    // }
  }

  public void periodic() {
    // Update System Alerts
    // Check CAN status
    // TODO check against REV backend API
    var canStatus = RobotController.getCANStatus();
    canErrorAlert.set(
        canErrorDebouncer.calculate(
            canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0));

    // Update Battery Voltage Alert
    lowBatteryVoltageAlert.set(
        batteryVoltageDebouncer.calculate(
            RobotController.getBatteryVoltage() <= LOW_VOLTAGE_WARNING_THRESHOLD));

    // Update Program Alerts
    // TODO

    // // Update Hardware Alerts
    // if (leds == null) return;
    //
    // // Disable LEDs if battery voltage is too low
    // if(ledVoltageDebouncer.calculate(
    //     RobotController.getBatteryVoltage() <= LED_DISABLE_VOLTAGE_THRESHOLD)) {
    //   ledsDisabledAlert.set(true);
    //   ledLowVoltageDisabledAlert.set(true);
    //
    //   leds.stop();
    //   leds.close();
    //
    //   leds = null;
    //   ledBuffer = null;
    //
    //   return;
  }

  // LEDs have a base mode / pattern / effect
  // alerts will update and override specific sections if something is active
  // apply the buffer

  // private static class HardwareIndicatedAlert extends Alert {
  //   /**
  //    * Creates a new alert in the default group - "Alerts". If this is the first to be
  // instantiated,
  //    * the appropriate entries will be added to NetworkTables.
  //    *
  //    * @param text Text to be displayed when the alert is active.
  //    * @param type Alert urgency level.
  //    */
  //   public HardwareIndicatedAlert(String text, AlertType type) {
  //     super(text, type);
  //   }
  // }
}
