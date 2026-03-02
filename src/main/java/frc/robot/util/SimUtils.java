package frc.robot.util;

public class SimUtils {
  private static double DT = 0.02; // 20 ms step

  public static double slew(double current, double target, double accelRate) {
    double step = accelRate * DT;
    double error = target - current;
    if (Math.abs(error) < step) {
      return target;
    }
    return current + Math.signum(error) * step;
  }
}
