package frc.robot.util;

import edu.wpi.first.networktables.ConnectionInfo;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.nio.ByteBuffer;
import java.util.HashSet;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/** Utility class to log the list of NetworkTables clients. */
public class NTClientLogger {
  private static final String tableName = "NTClients/";
  private static Set<String> lastRemoteIds = new HashSet<>();
  private static final ByteBuffer intBuffer = ByteBuffer.allocate(4);

  private NTClientLogger() {}

  public static void log() {
    ConnectionInfo[] connections = NetworkTableInstance.getDefault().getConnections();
    Set<String> remoteIds = new HashSet<>();

    // Log data for connected clients
    for (ConnectionInfo connection : connections) {
      lastRemoteIds.remove(connection.remote_id);
      remoteIds.add(connection.remote_id);
      Logger.recordOutput(tableName + connection.remote_id + "/Connected", true);
      Logger.recordOutput(tableName + connection.remote_id + "/IPAddress", connection.remote_ip);
      Logger.recordOutput(tableName + connection.remote_id + "/RemotePort", connection.remote_port);
      Logger.recordOutput(tableName + connection.remote_id + "/LastUpdate", connection.last_update);
      intBuffer.rewind();
      Logger.recordOutput(
          tableName + connection.remote_id + "/ProtocolVersion",
          intBuffer.putInt(connection.protocol_version).array());
    }

    // Mark disconnected clients
    for (var remoteId : lastRemoteIds) {
      Logger.recordOutput(tableName + remoteId + "/Connected", false);
    }
    lastRemoteIds = remoteIds;
  }
}
