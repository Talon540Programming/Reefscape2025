package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
  private final Elevator elevator;
  private final Effector effector;

  public Superstructure(ElevatorIO elevatorIO, EffectorIO effectorIO) {
    elevator = new Elevator(elevatorIO);
    effector = new Effector(effectorIO);
  }

  @Override
  public void periodic() {
    // Run periodic
    elevator.periodic();
    effector.periodic();

    // Safety? TODO
    // State check? TODO
  }
}
