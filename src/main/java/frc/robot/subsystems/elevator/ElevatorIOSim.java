package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {
  private final DCMotor elevatorMotorModel = DCMotor.getNEO(2);
  private final ElevatorSim sim =
      new ElevatorSim(
          elevatorMotorModel,
          gearing,
          carriageMassKg + stagesMassKg,
          drumRadius,
          0,
          elevatorMaxTravel,
          true,
          0,
          0.0035,
          0.0);

  private final PIDController controller = new PIDController(0, 0, 0);

  private double appliedVolts = 0.0;
  private boolean closedLoop = false;
  private double feedforward = 0.0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (!closedLoop) {
      controller.reset();
    } else {
      appliedVolts = controller.calculate(sim.getPositionMeters()) + feedforward;
    }

    sim.setInputVoltage(appliedVolts);
    sim.update(Constants.kLoopPeriodSecs);

    inputs.leaderConnected = true;
    inputs.followerConnected = true;

    inputs.positionRads = sim.getPositionMeters() / drumRadius;
    inputs.velocityRadPerSec = sim.getVelocityMetersPerSecond() / drumRadius;

    inputs.appliedVolts = new double[] {appliedVolts};
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
  }

  @Override
  public void runOpenLoop(double output) {
    closedLoop = false;
    appliedVolts = output;
  }

  @Override
  public void runPosition(double positionRads, double feedforwardVolts) {
    closedLoop = true;
    controller.setSetpoint(positionRads);
    feedforward = feedforwardVolts;
  }

  @Override
  public void stop() {
    runOpenLoop(0);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    controller.setPID(kP, kI, kD);
  }
}
