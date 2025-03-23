package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  private static RobotType robotType = RobotType.SIMBOT;
  // Allows tunable values to be changed when enabled. Also adds tunable selectors to AutoSelector
  public static final boolean TUNING_MODE = true;
  // Disable the AdvantageKit logger from running
  public static final boolean ENABLE_LOGGING = true;
  // Disable LEDs, will reduce software and electrical overhead but disable hardware alerts
  public static final boolean ENABLE_LEDs = false;

  public static final double kLoopPeriodSecs = 0.02;

  public static RobotType getRobot() {
    if (RobotBase.isReal() && robotType == RobotType.SIMBOT) {
      new Alert(
              "Invalid robot selected, using competition robot as default.", Alert.AlertType.kError)
          .set(true);
      robotType = RobotType.COMPBOT;
    }
    return robotType;
  }

  public static Mode getMode() {
    return switch (robotType) {
      case COMPBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum RobotType {
    SIMBOT,
    COMPBOT
  }
}
