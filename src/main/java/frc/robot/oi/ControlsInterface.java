package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ControlsInterface {
  public double getDriveX();

  public double getDriveY();

  public double getDriveTheta();

  public Trigger robotRelativeOverride();

  public Trigger stopWithX();

  public Trigger resetGyro();

  public Trigger depositL1();

  public Trigger depositL2left();

  public Trigger depositL2right();

  public Trigger removeL2Algae();

  public Trigger depositL3left();

  public Trigger depositL3right();

  public Trigger removeL3Algae();

  public Trigger intake();
}
