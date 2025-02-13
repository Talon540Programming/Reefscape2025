package frc.robot.util;

import edu.wpi.first.wpilibj.RobotBase;
import frc.generated.BuildConstants;
import frc.robot.Constants;
import java.nio.file.Path;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class LoggerUtil {
  /** Initialize the Logger with the auto-generated data from the build. */
  public static void initializeLoggerMetadata() {
    // Record metadata from generated state file.
    Logger.recordMetadata("ROBOT_NAME", Constants.getRobot().toString());
    Logger.recordMetadata("RUNTIME_ENVIRONMENT", RobotBase.getRuntimeType().toString());
    Logger.recordMetadata("TUNING_MODE", Boolean.toString(Constants.TUNING_MODE));
    Logger.recordMetadata("PROJECT_NAME", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BUILD_DATE", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GIT_SHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GIT_DATE", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GIT_BRANCH", BuildConstants.GIT_BRANCH);

    // Set the current GIT state of the robot (helps manage the logs that are saved).
    switch (BuildConstants.DIRTY) {
      case 0 -> Logger.recordMetadata("GIT_STATUS", "All changes committed");
      case 1 -> Logger.recordMetadata("GIT_STATUS", "Uncommitted changes");
      default -> Logger.recordMetadata("GIT_STATUS", "Unknown");
    }
  }

  /**
   * Get the path to the logging directory. Returns Empty if the USB drive is not plugged into the
   * robot.
   *
   * @return logging path. Empty if the drive is not plugged in.
   */
  public static Optional<Path> getLogPath() {
    var usbPath = Path.of("/U/logs");
    // Return USB path if it is plugged in
    if (usbPath.toFile().exists()) {
      return Optional.of(usbPath);
    }

    return Optional.empty();
  }
}
