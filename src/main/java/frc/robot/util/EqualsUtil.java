package frc.robot.util;

import edu.wpi.first.math.geometry.Twist2d;

public class EqualsUtil {
  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, 1e-9);
  }

  /** Extension methods for wpi geometry objects */
  public static class GeomExtensions {
    public static boolean epsilonEquals(Twist2d twist, Twist2d other) {
      return EqualsUtil.epsilonEquals(twist.dx, other.dx)
          && EqualsUtil.epsilonEquals(twist.dy, other.dy)
          && EqualsUtil.epsilonEquals(twist.dtheta, other.dtheta);
    }

    public static boolean equalsZero(Twist2d twist) {
      return EqualsUtil.epsilonEquals(twist.dx, 0.0)
          && EqualsUtil.epsilonEquals(twist.dy, 0.0)
          && EqualsUtil.epsilonEquals(twist.dtheta, 0.0);
    }
  }
}
