package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.util.LoggedTracer;
import frc.robot.util.VirtualSubsystem;
import java.util.List;
import java.util.function.BiConsumer;

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
  private static final double rainbowStrobeDuration = 0.2;
  private static final double waveExponent = 0.4;
  private static final double waveFastCycleLength = 25.0;
  private static final double waveFastDuration = 0.25;
  private static final double waveDisabledCycleLength = 15.0;
  private static final double waveDisabledDuration = 2.0;
  private static final double autoFadeTime = 2.5; // 3s nominal
  private static final double autoFadeMaxTime = 5.0; // Return to normal
  private static final Color l1Color = Color.kOrange;
  private static final Color l2Color = Color.kYellow;
  private static final Color l3Color = Color.kGreen;
  private static final Color l4Color = Color.kPurple;

  // Color Constants
  private static final Color primaryColor = Color.kWhite;
  private static Color disabledColor = Color.kGold;
  private static Color secondaryDisabledColor = Color.kRed;

  // LED IO
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private final Notifier loadingNotifier;

  // LED Sections
  private final Section fullSection = new Section(0, numLEDs);
  private final Section bottomSection = new Section(0, 31);
  private final Section topSideSection = null; // TODO need to wire this
  private final Section topSection = null; // TODO need to wire this

  // State Constants
  private static final int minLoopCycleCount = 10;
  public int loopCycleCount = 0;
  public boolean humanPlayerAlert = false;
  public boolean lowBatteryAlert = false;
  public boolean intaking = false;
  public boolean endgameAlert = false;
  public boolean robotTipping = false;
  public boolean autoScoringReef = false;
  public ReefLevel autoScoringLevel = null;
  public boolean visionDisconnected = false;
  private boolean lastEnabledAuto = false;
  private double lastEnabledTime = 0.0;
  public boolean coralGrabbed = false;

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
    loadingNotifier.setName("LEDs Loading Indicator");
    loadingNotifier.startPeriodic(0.02);
  }

  @Override
  public synchronized void periodic() {
    // Update alliance color
    if (DriverStation.isFMSAttached()) {
      var allianceOpt = DriverStation.getAlliance();
      allianceOpt.ifPresent(
          alliance -> {
            disabledColor = alliance == DriverStation.Alliance.Blue ? Color.kBlue : Color.kRed;
            secondaryDisabledColor = Color.kBlack;
          });
    }

    // Update auto state
    if (DriverStation.isEnabled()) {
      lastEnabledAuto = DriverStation.isAutonomous();
      lastEnabledTime = Timer.getTimestamp();
    }

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
      if (lastEnabledAuto && Timer.getTimestamp() - lastEnabledTime < autoFadeMaxTime) {
        // Auto fade
        wave(
            new Section(
                (int)
                    (bottomSection.end * ((Timer.getTimestamp() - lastEnabledTime) / autoFadeTime)),
                bottomSection.end),
            Color.kRed,
            Color.kWhite,
            waveFastCycleLength,
            waveFastDuration);
      } else if (lowBatteryAlert) {
        // Low battery
        strobe(fullSection, Color.kOrangeRed, Color.kBlack, strobeDuration);
      } else {
        // Default pattern
        wave(
            fullSection,
            disabledColor,
            secondaryDisabledColor,
            waveDisabledCycleLength,
            waveDisabledDuration);
      }

      // Vision disconnected alert
      if (visionDisconnected) {
        strobe(topSection, Color.kRed, Color.kBlack, strobeDuration);
      }
    } else if (DriverStation.isAutonomous()) {
      wave(fullSection, Color.kRed, Color.kWhite, waveFastCycleLength, waveFastDuration);
    } else {
      // Default pattern
      rainbow(fullSection, rainbowCycleLength, rainbowDuration);

      if (autoScoringReef) {
        rainbow(topSection, rainbowCycleLength, rainbowDuration);
        solid(
            topSideSection,
            switch (autoScoringLevel) {
              case L1 -> l1Color;
              case L2 -> l2Color;
              case L3 -> l3Color;
              case L4 -> l4Color;
            });
      }

      if (robotTipping) {
        strobe(fullSection, Color.kRed, Color.kBlack, strobeDuration);
      }

      if (intaking) {
        strobe(fullSection, Color.kRed, Color.kBlue, strobeDuration);
      }

      if (coralGrabbed) {
        solid(topSection, Color.kLime);
      }

      // Human player alert
      if (humanPlayerAlert) {
        strobe(fullSection, Color.kWhite, Color.kBlack, strobeDuration);
      }

      // Endgame alert
      if (endgameAlert) {
        strobe(fullSection, Color.kRed, Color.kWhite, strobeDuration);
      }
    }

    // Update Followed Sections
    for (int i = bottomSection.start; i < bottomSection.end; i++) {
      buffer.setLED(61 - i, buffer.getLED(i));
    }
    // TODO
    // for(int i = topSideSection.start; i < topSideSection.end; i++) {}

    // Set buffer to LEDs
    leds.setData(buffer);

    // Record cycle time
    LoggedTracer.record("LEDs");
  }

  public static Trigger setLEDState(Trigger base, BiConsumer<LEDBase, Boolean> stateSetter) {
    return base.onTrue(Commands.runOnce(() -> stateSetter.accept(LEDBase.getInstance(), true)))
        .onFalse(Commands.runOnce(() -> stateSetter.accept(LEDBase.getInstance(), false)));
  }

  public static Command setLEDState(Command base, BiConsumer<LEDBase, Boolean> stateSetter) {
    return base.deadlineFor(
        Commands.startEnd(
            () -> stateSetter.accept(LEDBase.getInstance(), true),
            () -> stateSetter.accept(LEDBase.getInstance(), false)));
  }

  private void solid(Section section, Color color) {
    if (section == null || color == null) return;
    for (int ledIdx = section.start(); ledIdx < section.end(); ledIdx++) {
      buffer.setLED(ledIdx, color);
    }
  }

  private void strobe(Section section, Color c1, Color c2, double duration) {
    if (section == null) return;
    boolean c1On = ((Timer.getTimestamp() % duration) / duration) > 0.5;
    solid(section, c1On ? c1 : c2);
  }

  private void breath(Section section, Color c1, Color c2, double duration, double timestamp) {
    if (section == null) return;
    double x = ((timestamp % duration) / duration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    var color = Color.lerpRGB(c1, c2, ratio);
    solid(section, color);
  }

  private void rainbow(Section section, double cycleLength, double duration) {
    if (section == null) return;
    double base = (1 - ((Timer.getTimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int ledIdx = section.end() - 1; ledIdx >= section.start(); ledIdx--) {
      double x = (base + ledIdx * xDiffPerLed) % 180.0;
      buffer.setHSV(ledIdx, (int) x, 255, 255);
    }
  }

  private void wave(Section section, Color c1, Color c2, double cycleLength, double duration) {
    if (section == null) return;
    double base = (1 - ((Timer.getTimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int ledIdx = section.end() - 1; ledIdx >= section.start(); ledIdx--) {
      double x = base + ledIdx * xDiffPerLed;
      double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;

      if (Double.isNaN(ratio)) {
        ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
      }
      if (Double.isNaN(ratio)) {
        ratio = 0.5;
      }

      var color = Color.lerpRGB(c1, c2, ratio);
      buffer.setLED(ledIdx, color);
    }
  }

  private void stripes(Section section, List<Color> colors, int stripeLength, double duration) {
    if (section == null) return;
    int offset = (int) (Timer.getTimestamp() % duration / duration * stripeLength * colors.size());
    for (int ledIdx = section.end() - 1; ledIdx >= section.start(); ledIdx--) {
      int colorIndex =
          (int) (Math.floor((double) (ledIdx - offset) / stripeLength) + colors.size())
              % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      buffer.setLED(ledIdx, colors.get(colorIndex));
    }
  }

  private record Section(int start, int end) {}
}
