package frc.robot.oi;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SimKeyboard implements ControlsInterface {
  private final GenericHID hid;

  public SimKeyboard(int port) {
    hid = new GenericHID(port);
  }

  @Override
  public double getDriveX() {
    return -hid.getRawAxis(0);
  }

  @Override
  public double getDriveY() {
    return -hid.getRawAxis(1);
  }

  @Override
  public double getDriveTheta() {
    return -hid.getRawAxis(2);
  }

  @Override
  public Trigger robotRelativeOverride() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger depositL1() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger depositL2left() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger depositL2right() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger removeL2Algae() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger depositL3left() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger depositL3right() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger removeL3Algae() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger intake() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger enableLeftReefAutoAlignment() {
    return new Trigger(() -> hid.getRawButton(0));
  }

  @Override
  public Trigger enableRightReefAutoAlignment() {
    return new Trigger(() -> hid.getRawButton(1));
  }

  @Override
  public Trigger stopWithX() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger resetGyro() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger slowMode() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger back() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger stow() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger L1() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger L2() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger L3() {
    return new Trigger(() -> false);
  }
}
