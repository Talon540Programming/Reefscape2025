package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.*;
import frc.robot.util.VirtualSubsystem;

public class LEDBase extends VirtualSubsystem {
  private static LEDBase instance;

  public static LEDBase getInstance() {
    if (instance == null) {
      instance = new LEDBase();
    }
    return instance;
  }

  // LED IO
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;

  // LED Sections
  private final AddressableLEDBufferView leftIntake;
  private final AddressableLEDBufferView rightIntake;
  // private final AddressableLEDBufferView firstTopBar;
  // private final AddressableLEDBufferView secondTopBar;

  // Constants
  private static final int numLEDs = 62;
  private static final int ledDriverPort = 0;

  private LEDBase() {
    // Initialize IO
    leds = new AddressableLED(ledDriverPort);
    buffer = new AddressableLEDBuffer(numLEDs);

    // Create Section Views
    leftIntake = buffer.createView(0, 30);
    rightIntake = buffer.createView(31, 61);

    leds.setLength(numLEDs);
    leds.setData(buffer);

    leds.start();
  }

  private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 255);
  private final LEDPattern m_scrollingRainbow =
      m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meters.of(1.0 / 60));

  @Override
  public synchronized void periodic() {
    m_scrollingRainbow.applyTo(buffer);
    leds.setData(buffer);
  }
}
