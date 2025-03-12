package frc.robot.util;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;

// LED Alerts
// endgameAlert
// autoScoring (Reef Side denotes side, color denotes level)
// superstructureEstopped
// lowBatteryAlert
// visionDisconnected
// following trajectory
// go to pose
// feederstation alert (alert feeder station player about being OTW)

// TODO indicate robot initializing in LEDs

public class AlertsUtil {
  private static final int numLeds = 120;
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
  private final Alert joystickDisconnectedAlert =
      new Alert("At least one joystick button is not detected", AlertType.kWarning);
  private final Debouncer joystickDebouncer = new Debouncer(2);


  // Program Alerts
  private final Alert tuningModeAlert = new Alert("Robot in Tuning Mode", AlertType.kInfo);
  private final Alert ledsDisabledAlert =
      new Alert("LEDs disabled by program override", AlertType.kInfo);

  private AddressableLED leds;
  private AddressableLEDBuffer ledBuffer;

  private AlertsUtil() {
    if (Constants.TUNING_MODE) {
      tuningModeAlert.set(true);
    }

    if (Constants.ENABLE_LEDs) {
      leds = new AddressableLED(1);
      ledBuffer = new AddressableLEDBuffer(numLeds);

      leds.setLength(ledBuffer.getLength());
      leds.start();
    } else {
      ledsDisabledAlert.set(true);
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

    joystickDisconnectedAlert.set(
        joystickDebouncer.calculate(DriverStation.isJoystickConnected(0)));


    // Update Program Alerts
    // TODO

    // Update Hardware Alerts
    if (leds == null) return;

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

  // private Color solid(Section section, Color color) {
  //   if (color != null) {
  //     for (int i = section.start(); i < section.end(); i++) {
  //       ledBuffer.setLED(i, color);
  //     }
  //   }
  //   return color;
  // }
  //
  // private Color strobe(Section section, Color c1, Color c2, double duration) {
  //   boolean c1On = ((Timer.getTimestamp() % duration) / duration) > 0.5;
  //   return solid(section, c1On ? c1 : c2);
  // }
  //
  // private Color breath(Section section, Color c1, Color c2, double duration, double timestamp) {
  //   double x = ((timestamp % duration) / duration) * 2.0 * Math.PI;
  //   double ratio = (Math.sin(x) + 1.0) / 2.0;
  //   double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
  //   double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
  //   double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
  //   var color = new Color(red, green, blue);
  //   solid(section, color);
  //   return color;
  // }
  //
  // private Color breath(Section section, Color c1, Color c2, double duration) {
  //   return breath(section, c1, c2, duration, Timer.getTimestamp());
  // }
  //
  // private void rainbow(Section section, double cycleLength, double duration) {
  //   double x = (1 - ((Timer.getTimestamp() / duration) % 1.0)) * 180.0;
  //   double xDiffPerLed = 180.0 / cycleLength;
  //   for (int i = section.end() - 1; i >= section.start(); i--) {
  //     x += xDiffPerLed;
  //     x %= 180.0;
  //     ledBuffer.setHSV(i, (int) x, 255, 255);
  //   }
  // }
  //
  // private void wave(Section section, Color c1, Color c2, double cycleLength, double duration) {
  //   double x = (1 - ((Timer.getTimestamp() % duration) / duration)) * 2.0 * Math.PI;
  //   double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
  //   for (int i = section.end() - 1; i >= section.start(); i--) {
  //     x += xDiffPerLed;
  //     double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
  //     if (Double.isNaN(ratio)) {
  //       ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
  //     }
  //     if (Double.isNaN(ratio)) {
  //       ratio = 0.5;
  //     }
  //     double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
  //     double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
  //     double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
  //     ledBuffer.setLED(i, new Color(red, green, blue));
  //   }
  // }
  //
  // private void stripes(Section section, List<Color> colors, int stripeLength, double duration) {
  //   int offset = (int) (Timer.getTimestamp() % duration / duration * stripeLength *
  // colors.size());
  //   for (int i = section.end() - 1; i >= section.start(); i--) {
  //     int colorIndex =
  //         (int) (Math.floor((double) (i - offset) / stripeLength) + colors.size()) %
  // colors.size();
  //     colorIndex = colors.size() - 1 - colorIndex;
  //     ledBuffer.setLED(i, colors.get(colorIndex));
  //   }
  // }

  // LEDs have a base mode / pattern / effect
  // alerts will update and override specific sections if something is active
  // apply the buffer

  public static class HardwareIndicatedAlert extends Alert {
    public HardwareIndicatedAlert(
        String text, AlertType type, int priority /*TODO handle LED stuff*/) {
      super(text, type);
    }
  }

  private static record Section(int start, int end) {}
}
