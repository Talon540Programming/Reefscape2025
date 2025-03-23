package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.VirtualSubsystem;
import java.util.List;

public class LEDBase extends VirtualSubsystem {
  private static LEDBase instance;

  public static LEDBase getInstance() {
    if (instance == null) {
      instance = new LEDBase();
    }
    return instance;
  }

  // LED Constants
  private static final int numLEDs = 62;
  private static final int ledDriverPort = 0;
  private static final double strobeDuration = 0.2;
  private static final double breathFastDuration = 0.5;
  private static final double breathSlowDuration = 1.0;
  private static final double rainbowCycleLength = 25.0;
  private static final double rainbowDuration = 0.25;
  private static final double waveExponent = 0.4;
  private static final double waveFastCycleLength = 25.0;
  private static final double waveFastDuration = 0.25;
  private static final double waveDisabledCycleLength = 15.0;
  private static final double waveDisabledDuration = 2.0;
  private static final double autoFadeTime = 2.5; // 3s nominal
  private static final double autoFadeMaxTime = 5.0; // Return to normal

  // Color Constants
  private static final Color primaryColor = Color.kWhite;
  private static final Color secondaryColor = null; // TODO
  private static final Color disabledColor = null; // TODO
  private static final Color secondaryDisabledColor = null; // TODO
  private static final Color stowColor = null; // TODO
  private static final Color l1Color = Color.kHotPink;
  private static final Color l2Color = Color.kOrange;
  private static final Color l3Color = Color.kYellow;
  private static final Color l4Color = Color.kGreen;

  // LED IO
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private final Notifier loadingNotifier;

  // LED Sections
  private final Section fullSection = new Section(0, numLEDs);
  private final Section leftBottomSection = new Section(0, 30);
  private final Section rightBottomSection = new Section(31, 61);
  private final Section firstTopBarSection = null; // TODO
  private final Section secondTopBarSection = null; // TODO

  // private final Section scoringStateIndicators = null; // TODO
  // private final Section coralIndicator = null; // TODO
  // private final Section humanPlayerIndicator = null; // TODO

  // State Constants
  private static final int minLoopCycleCount = 10;
  public int loopCycleCount = 0;
  public boolean autoScoringReef = false;
  public boolean humanPlayerAlert = false;
  public boolean endgameAlert = false;

  private LEDBase() {
    // Initialize IO
    leds = new AddressableLED(ledDriverPort);
    buffer = new AddressableLEDBuffer(numLEDs);

    leds.setLength(numLEDs);
    leds.setData(buffer);

    leds.start();

    loadingNotifier =
        new Notifier(
            () -> {
              synchronized (this) {
                breath(
                    fullSection,
                    primaryColor,
                    Color.kBlack,
                    breathSlowDuration,
                    Timer.getTimestamp());
                leds.setData(buffer);
              }
            });
    loadingNotifier.startPeriodic(0.02);
  }

  @Override
  public synchronized void periodic() {
    // Exit during initial cycles
    loopCycleCount += 1;
    if (loopCycleCount < minLoopCycleCount) {
      return;
    }

    // Stop loading notifier if running
    loadingNotifier.stop();

    // Select LED mode
    solid(fullSection, Color.kBlack); // Default to off
    if (DriverStation.isEStopped()) {
      solid(fullSection, Color.kRed);
    } else if (DriverStation.isDisabled()) {
      // Default pattern
      rainbow(fullSection, rainbowCycleLength, rainbowDuration);
    } else if (DriverStation.isAutonomous()) {
      wave(fullSection, Color.kRed, Color.kWhite, waveFastCycleLength, waveFastDuration);
    } else {
      // Default pattern
      rainbow(fullSection, rainbowCycleLength, rainbowDuration);

      if (autoScoringReef) {
        wave(fullSection, Color.kRed, Color.kOrange, waveFastCycleLength, waveFastDuration);
      }

      // Human player alert
      if (humanPlayerAlert) {
        strobe(fullSection, Color.kRed, Color.kBlue, strobeDuration);
      }

      // Endgame alert
      if (endgameAlert) {
        strobe(fullSection, Color.kRed, Color.kWhite, strobeDuration);
      }
    }

    // Set buffer to LEDs
    leds.setData(buffer);
  }

  private void solid(Section section, Color color) {
    if (color != null) {
      for (int i = section.start(); i < section.end(); i++) {
        buffer.setLED(i, color);
      }
    }
  }

  private void strobe(Section section, Color c1, Color c2, double duration) {
    boolean c1On = ((Timer.getTimestamp() % duration) / duration) > 0.5;
    solid(section, c1On ? c1 : c2);
  }

  private void breath(Section section, Color c1, Color c2, double duration, double timestamp) {
    double x = ((timestamp % duration) / duration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    var color = Color.lerpRGB(c1, c2, ratio);
    solid(section, color);
  }

  private void rainbow(Section section, double cycleLength, double duration) {
    double base = (1 - ((Timer.getTimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = section.end() - 1; i >= section.start(); i--) {
      double x = (base + i * xDiffPerLed) % 180.0;
      buffer.setHSV(i, (int) x, 255, 255);
    }
  }

  private void wave(Section section, Color c1, Color c2, double cycleLength, double duration) {
    double base = (1 - ((Timer.getTimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = section.end() - 1; i >= section.start(); i--) {
      double x = base + i * xDiffPerLed;
      double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;

      if (Double.isNaN(ratio)) {
        ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
      }
      if (Double.isNaN(ratio)) {
        ratio = 0.5;
      }

      var color = Color.lerpRGB(c1, c2, ratio);
      buffer.setLED(i, color);
    }
  }

  private void stripes(Section section, List<Color> colors, int stripeLength, double duration) {
    int offset = (int) (Timer.getTimestamp() % duration / duration * stripeLength * colors.size());
    for (int i = section.end() - 1; i >= section.start(); i--) {
      int colorIndex =
          (int) (Math.floor((double) (i - offset) / stripeLength) + colors.size()) % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      buffer.setLED(i, colors.get(colorIndex));
    }
  }

  private record Section(int start, int end) {}
}
