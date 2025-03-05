package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.PigeonConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Queue;

public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 m_gyro = new Pigeon2(id);

  private final StatusSignal<Angle> yaw;
  private final StatusSignal<AngularVelocity> yawVelocity;

  private final Queue<Double> yawPositionQueue;

  public GyroIOPigeon2() {
    m_gyro.getConfigurator().apply(new Pigeon2Configuration());
    m_gyro.getConfigurator().setYaw(0.0);

    yaw = m_gyro.getYaw();
    yawVelocity = m_gyro.getAngularVelocityZWorld();

    yaw.setUpdateFrequency(DriveConstants.odometryFrequencyHz);
    yawVelocity.setUpdateFrequency(50.0);
    m_gyro.optimizeBusUtilization();

    yawPositionQueue =
        OdometryManager.getInstance().registerSignal(() -> m_gyro.getYaw().getValueAsDouble());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);

    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawPositions =
        yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
    yawPositionQueue.clear();
  }
}