package frc.robot.subsystems.effector;

import static frc.robot.subsystems.effector.EffectorConstants.*;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class RollerIOSim implements RollerIO {
  private final FlywheelSim sim;

  private double appliedVolts = 0.0;

  public RollerIOSim() {
    sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(rollerMotorModel, rollerMoI, rollerGearing),
            rollerMotorModel);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    sim.setInputVoltage(appliedVolts);
    sim.update(Constants.kLoopPeriodSecs);

    inputs.appliedVolts = appliedVolts;
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.currentAmps = sim.getCurrentDrawAmps();
    inputs.connected = true;
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
