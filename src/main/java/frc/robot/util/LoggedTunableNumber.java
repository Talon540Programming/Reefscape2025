package frc.robot.util;

import frc.robot.Constants;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class LoggedTunableNumber implements DoubleSupplier {
  private static final String tableKey = "/TunableNumbers";

  private final String key;
  private Double defaultValue = null;

  private final Map<Integer, Double> lastValues = new HashMap<>();

  private LoggedNetworkNumber dashboardNumber;

  /**
   * Create a new LoggedTunableNumber
   *
   * @param dashboardKey Key on dashboard
   */
  public LoggedTunableNumber(String dashboardKey) {
    this.key = tableKey + "/" + dashboardKey;
  }

  /**
   * Create a new LoggedTunableNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    initDefault(defaultValue);
  }

  /**
   * Set the default value of the number. The default value can only be set once.
   *
   * @param defaultValue The default value
   * @throws IllegalStateException If a default value has already been set either by the constructor
   *     or by a previous call to this method.
   */
  public void initDefault(double defaultValue) {
    if (this.defaultValue != null) {
      throw new IllegalStateException(
          String.format(
              "[LoggedTunableNumber][%s] Has already been initialized with a default value.", key));
    }

    this.defaultValue = defaultValue;
    if (Constants.TUNING_MODE) {
      dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @return The current value
   * @throws IllegalStateException If a default value hasn't been set yet.
   */
  public double get() {
    if (defaultValue == null) {
      throw new IllegalStateException(
          String.format(
              "[LoggedTunableNumber][%s] Hasn't been initialized with a default value. Make sure to call initDefault or use the correct constructor.",
              key));
    }

    return Constants.TUNING_MODE ? dashboardNumber.get() : defaultValue;
  }

  /**
   * Checks whether the number has changed since the last time this method was called. Returns true
   * the first time this method is called.
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @return Whether the value has changed since the last time this method was called
   */
  public boolean hasChanged(int id) {
    double currentValue = get();
    var lastValue = lastValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastValues.put(id, currentValue);
      return true;
    }

    return false;
  }

  /**
   * Checks whether the number has changed since the last time this method was called. Returns true
   * the first time this method is called.
   *
   * @apiNote This method assumes that there is only a single object is calling this method. For
   *     that use case, see {@link #hasChanged(int)}.
   * @return Whether the value has changed since the last time this method was called
   */
  public boolean hasChanged() {
    return hasChanged(0);
  }

  /**
   * Run callback if any tunable number has changed. See {@link #hasChanged(int)} for usage.
   *
   * @param action action to run
   * @param tunableNumbers tunable numbers to check
   */
  public static void ifChanged(int id, Runnable action, LoggedTunableNumber... tunableNumbers) {
    if (Arrays.stream(tunableNumbers).anyMatch(v -> v.hasChanged(id))) {
      action.run();
    }
  }

  /**
   * Run callback if any tunable number has changed. See {@link #hasChanged()} for usage.
   *
   * @param action action to run
   * @param tunableNumbers tunable numbers to check
   */
  public static void ifChanged(Runnable action, LoggedTunableNumber... tunableNumbers) {
    if (Arrays.stream(tunableNumbers).anyMatch(LoggedTunableNumber::hasChanged)) {
      action.run();
    }
  }

  @Override
  public double getAsDouble() {
    return get();
  }
}
