// Modified from WPILib's SlewRateLimiter:
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.util;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * A class that limits the rate of change of an input value. Useful for implementing voltage,
 * setpoint, and/or output ramps. A slew-rate limit is most appropriate when the quantity being
 * controlled is a velocity or a voltage; when controlling a position, consider using a {@link
 * edu.wpi.first.math.trajectory.TrapezoidProfile} instead.
 */
public class LoggedSlewRateLimiter {
  private final LoggedNetworkNumber loggedNumber;

  private double rateLimit;
  private double prevVal;
  private double prevTime;

  /**
   * Creates a new SlewRateLimiter with the given positive rate limit and negative rate limit of
   * -rateLimit.
   *
   * @param rateLimit The rate-of-change limit, in units per second.
   */
  public LoggedSlewRateLimiter(double rateLimit, String ntKey) {
    this.rateLimit = rateLimit;
    prevVal = 0;
    this.loggedNumber = new LoggedNetworkNumber(ntKey, rateLimit);
    prevTime = MathSharedStore.getTimestamp();
  }

  public void periodic() {
    if (this.loggedNumber.get() != this.rateLimit) {
      this.rateLimit = this.loggedNumber.get();
    }
  }

  /**
   * Filters the input to limit its slew rate.
   *
   * @param input The input value whose slew rate is to be limited.
   * @return The filtered value, which will not change faster than the slew rate.
   */
  public double calculate(double input) {
    // We are decelerating if we change sign or if the magnitude of the input is
    // less than the magnitude of the previous value
    var decelerating =
        Math.abs(input) < Math.abs(prevVal) || (Math.signum(input) * Math.signum(prevVal) < 0);

    double currentTime = MathSharedStore.getTimestamp();
    double elapsedTime = currentTime - prevTime;
    if (decelerating) {
      prevVal = input;
    } else {
      prevVal += MathUtil.clamp(input - prevVal, -rateLimit * elapsedTime, rateLimit * elapsedTime);
    }
    prevTime = currentTime;
    return prevVal;
  }

  /**
   * Returns the value last calculated by the SlewRateLimiter.
   *
   * @return The last value.
   */
  public double lastValue() {
    return prevVal;
  }

  /**
   * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
   *
   * @param value The value to reset to.
   */
  public void reset(double value) {
    prevVal = value;
    prevTime = MathSharedStore.getTimestamp();
  }
}
