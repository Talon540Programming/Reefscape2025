package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  private static RobotType kRobotType = RobotType.ROBOT_2025_COMP;
  // Allows tunable values to be changed when enabled. Also adds tunable selectors to AutoSelector
  public static final boolean TUNING_MODE = false;
  // Disable the AdvantageKit logger from running
  public static final boolean ENABLE_LOGGING = true;

  public static final double kLoopPeriodSecs = 0.02;

  public enum RobotMode {
    REAL,
    SIM,
    REPLAY
  }

  public enum RobotType {
    ROBOT_2025_COMP,
    ROBOT_SIMBOT
  }

  public static RobotType getRobotType() {
    if (RobotBase.isReal() && kRobotType == RobotType.ROBOT_SIMBOT) {
      DriverStation.reportError(
          "Robot is set to SIM but it isn't a SIM, setting it to Competition Robot as redundancy.",
          false);
      kRobotType = RobotType.ROBOT_2025_COMP;
    }

    if (RobotBase.isSimulation() && kRobotType != RobotType.ROBOT_SIMBOT) {
      DriverStation.reportError(
          "Robot is set to REAL but it is a SIM, setting it to SIMBOT as redundancy.", false);
      kRobotType = RobotType.ROBOT_SIMBOT;
    }

    return kRobotType;
  }

  public static RobotMode getRobotMode() {
    if (getRobotType() == RobotType.ROBOT_SIMBOT) return RobotMode.SIM;
    else return RobotBase.isReal() ? RobotMode.REAL : RobotMode.REPLAY;
  }
}
