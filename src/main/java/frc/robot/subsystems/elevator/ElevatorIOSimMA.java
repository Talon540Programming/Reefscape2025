// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.Constants;

public class ElevatorIOSimMA implements ElevatorIO {
  public static final DCMotor gearbox = DCMotor.getNEO(2).withReduction(kElevatorGearing);

  public static final Matrix<N2, N2> A =
      MatBuilder.fill(
          Nat.N2(),
          Nat.N2(),
          0,
          1,
          0,
          -gearbox.KtNMPerAmp
              / (gearbox.rOhms
                  * Math.pow(drumRadius, 2)
                  * (carriageMassKg + stagesMassKg)
                  * gearbox.KvRadPerSecPerVolt));
  public static final Vector<N2> B =
      VecBuilder.fill(0.0, gearbox.KtNMPerAmp / (drumRadius * (carriageMassKg + stagesMassKg)));

  // State given by elevator carriage position and velocity
  // Input given by torque current to motor
  private Vector<N2> simState;
  private double inputTorqueCurrent = 0.0;
  private double appliedVolts = 0.0;

  private final PIDController controller = new PIDController(Sim.kP, 0, Sim.kD);
  private boolean closedLoop = false;
  private double feedforward = 0.0;

  public ElevatorIOSimMA() {
    simState = VecBuilder.fill(0.0, 0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (!closedLoop) {
      controller.reset();
      update(Constants.kLoopPeriodSecs);
    } else {
      // Run control at 1khz
      for (int i = 0; i < Constants.kLoopPeriodSecs / (1.0 / 1000.0); i++) {
        setInputTorqueCurrent(controller.calculate(simState.get(0) / drumRadius) + feedforward);
        update(1.0 / 1000.0);
      }
    }

    inputs.positionRad = simState.get(0) / drumRadius;
    inputs.velocityRadPerSec = simState.get(1) / drumRadius;
    inputs.appliedVolts = new double[] {appliedVolts};
    inputs.currentAmps = new double[] {Math.copySign(inputTorqueCurrent, appliedVolts)};
  }

  public void runOpenLoop(double output) {
    closedLoop = false;
    setInputTorqueCurrent(output);
  }

  @Override
  public void stop() {
    runOpenLoop(0.0);
  }

  @Override
  public void runPosition(double positionRad, double feedforward) {
    closedLoop = true;
    controller.setSetpoint(positionRad);
    this.feedforward = feedforward;
  }

  private void setInputTorqueCurrent(double torqueCurrent) {
    inputTorqueCurrent = torqueCurrent;
    appliedVolts =
        gearbox.getVoltage(gearbox.getTorque(inputTorqueCurrent), simState.get(1, 0) / drumRadius);
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
  }

  private void setInputVoltage(double voltage) {
    setInputTorqueCurrent(gearbox.getCurrent(simState.get(1) / drumRadius, voltage));
  }

  private void update(double dt) {
    inputTorqueCurrent =
        MathUtil.clamp(inputTorqueCurrent, -gearbox.stallCurrentAmps, gearbox.stallCurrentAmps);
    Matrix<N2, N1> updatedState =
        NumericalIntegration.rkdp(
            (Matrix<N2, N1> x, Matrix<N1, N1> u) ->
                A.times(x)
                    .plus(B.times(u))
                    .plus(VecBuilder.fill(0.0, -Constants.G * Math.sin(elevatorAngle))),
            simState,
            MatBuilder.fill(Nat.N1(), Nat.N1(), inputTorqueCurrent),
            dt);
    // Apply limits
    simState = VecBuilder.fill(updatedState.get(0, 0), updatedState.get(1, 0));
    if (simState.get(0) <= 0.0) {
      simState.set(1, 0, 0.0);
      simState.set(0, 0, 0.0);
    }
    if (simState.get(0) >= maxElevatorHeightMeters) {
      simState.set(1, 0, 0.0);
      simState.set(0, 0, maxElevatorHeightMeters);
    }
  }
}
