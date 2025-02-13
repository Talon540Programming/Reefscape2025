package frc.robot.subsystems.effector;

import edu.wpi.first.math.system.plant.DCMotor;

public class EffectorConstants {
  public static final DCMotor indexerMotorModel = DCMotor.getNEO(1);
  public static final DCMotor rollerMotorModel = DCMotor.getNEO(1);

  public static final double indexerGearing = 4.0 / 1.0; // TODO
  public static final double indexerMoI = 0.0; // TODO

  public static final double rollerGearing = 0.0;
  public static final double rollerMoI = 0.0;

  public static final int indexerId = 0; // TODO
  public static final int rollerId = 0; // TODO
  public static final int beamBreakId = 0;

  public static final double indexerPositionConversion = 2 * Math.PI / indexerGearing;
  public static final double indexerVelocityConversion = (2 * Math.PI) / 60 / indexerGearing;

  public static final double rollerPositionConversion = 2 * Math.PI / indexerGearing;
  public static final double rollerVelocityConversion = (2 * Math.PI) / 60 / indexerGearing;
}
