package frc.robot.subsystems.reef_selector;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.util.VirtualSubsystem;
import java.nio.file.Path;

public class ReefSelectorBase extends VirtualSubsystem implements AutoCloseable {
  private static final int reefSelectorWebServerPort = 5801;
  private static final Path reefSelectorSrc =
      Filesystem.getDeployDirectory().toPath().resolve("reef_selector");

  private static final String rootTable = "/ReefSelector";

  private final IntegerArraySubscriber coralObjectiveSub;

  // private final IntegerArrayPublisher currentCoralObjectivePub;
  // private final BooleanPublisher isAutoscoringPub;

  public ReefSelectorBase() {
    // Create NT Topics
    var inputTable = NetworkTableInstance.getDefault().getTable(rootTable);
    coralObjectiveSub =
        inputTable
            .getIntegerArrayTopic("CoralObjective")
            .subscribe(new long[] {-1, -1}, PubSubOption.keepDuplicates(true));

    // Init WebServer
    WebServer.start(reefSelectorWebServerPort, reefSelectorSrc.toString());
    System.out.println(
        "[ReefSelector] Started Reef Selector Web Server on Port: " + reefSelectorWebServerPort);
  }

  @Override
  public void periodic() {}

  @Override
  public void close() {
    // Stop WebServer
    WebServer.stop(reefSelectorWebServerPort);

    // Shutdown NT Topics
    coralObjectiveSub.close();
  }
}
