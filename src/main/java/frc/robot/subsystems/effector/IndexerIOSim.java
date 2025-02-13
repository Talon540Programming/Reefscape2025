package frc.robot.subsystems.effector;

import static frc.robot.subsystems.effector.EffectorConstants.*;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class IndexerIOSim implements IndexerIO {
  private final FlywheelSim sim;

  private double appliedVolts = 0.0;

  public IndexerIOSim() {
    sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(indexerMotorModel, indexerMoI, indexerGearing),
            indexerMotorModel);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    sim.setInputVoltage(appliedVolts);
    sim.update(Constants.kLoopPeriodSecs);

    inputs.appliedVolts = appliedVolts;
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void runOpenLoop(double output) {
    appliedVolts = output;
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
  }
}
