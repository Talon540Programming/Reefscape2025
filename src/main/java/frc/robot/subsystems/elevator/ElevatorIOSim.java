package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.constants.Constants;

public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim sim;

  private boolean closedLoop = false;
  private PIDController controller = new PIDController(Sim.kP, 0, Sim.kD);
  private double FFVolts = 0.0;
  private double appliedVolts = 0.0;

  public ElevatorIOSim() {
    sim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                gearbox, carriageMassKg + stagesMassKg, drumRadius, kElevatorGearing),
            gearbox,
            minElevatorHeightMeters,
            maxElevatorHeightMeters,
            true,
            minElevatorHeightMeters);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (closedLoop) {
      appliedVolts = FFVolts + controller.calculate(sim.getPositionMeters() / drumRadius);
    } else {
      controller.reset();
    }

    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(Constants.kLoopPeriodSecs);

    inputs.connected = true;
    inputs.positionRad = sim.getPositionMeters() / drumRadius;
    inputs.velocityRadPerSec = sim.getVelocityMetersPerSecond();
    inputs.appliedVolts = new double[] {appliedVolts};
    inputs.currentAmps = new double[] {Math.abs(sim.getCurrentDrawAmps())};
  }

  @Override
  public void runOpenLoop(double output) {
    closedLoop = false;
    appliedVolts = output;
  }

  @Override
  public void runPosition(double positionRad, double feedfoward) {
    closedLoop = true;
    FFVolts = feedfoward;
    controller.setSetpoint(positionRad);
  }
}
