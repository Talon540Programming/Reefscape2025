package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.constants.Constants;

public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim m_sim;
  private final DCMotor m_motorModel = DCMotor.getNEO(2);
  private double appliedVoltage = 0.0;

  public ElevatorIOSim() {

    m_sim =
        new ElevatorSim(
            LinearSystemId.createDCMotorSystem(
                m_motorModel, ElevatorConstants.moi, ElevatorConstants.kElevatorGearing),
            m_motorModel,
            ElevatorConstants.minElevatorHeightMeters,
            ElevatorConstants.maxElevatorHeightMeters,
            true,
            ElevatorConstants.minElevatorHeightMeters,
            new double[] {0.0, 0.0});
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (DriverStation.isDisabled()) {}

    m_sim.update(Constants.kLoopPeriodSecs);
    inputs.positionMeters = m_sim.getPositionMeters();
    inputs.velocityRadPerSec = m_sim.getVelocityMetersPerSecond();
    inputs.appliedVolts = appliedVoltage;
    inputs.currentAmps = new double[] {m_sim.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double volts) {
    appliedVoltage = volts;
    m_sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void stop() {
    appliedVoltage = 0;
    m_sim.setInputVoltage(appliedVoltage);
  }
}
