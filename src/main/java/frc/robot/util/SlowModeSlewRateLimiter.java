package frc.robot.util;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class SlowModeSlewRateLimiter {
  private final LoggedNetworkNumber loggedPositiveRate;
  private final LoggedNetworkNumber loggedNegativeRate;

  private double positiveRateLimit;
  private double negativeRateLimit;
  private double prevVal;
  private double prevTime;

  /**
   * Creates a new SlewRateLimiter with the given positive rate limit and negative rate limit. The
   * slew rate limits are in units per second.
   *
   * @param positiveRateLimit The positive rate limit, in units per second (should be > 0).
   * @param negativeRateLimit The negative rate limit, in units per second (should be < 0).
   * @param initialValue The initial value to reset the limiter to.
   * @param ntKeyBase The base key for the network table values.
   */
  public SlowModeSlewRateLimiter(
      double positiveRateLimit, double negativeRateLimit, double initialValue, String ntKeyBase) {
    this.positiveRateLimit = positiveRateLimit;
    this.negativeRateLimit = negativeRateLimit;
    this.prevVal = initialValue;
    this.loggedPositiveRate = new LoggedNetworkNumber(ntKeyBase + "Up", positiveRateLimit);
    this.loggedNegativeRate = new LoggedNetworkNumber(ntKeyBase + "Down", negativeRateLimit);
    prevTime = MathSharedStore.getTimestamp();
  }

  public void periodic() {
    if (this.loggedPositiveRate.get() != this.positiveRateLimit) {
      this.positiveRateLimit = this.loggedPositiveRate.get();
    }
    if (this.loggedNegativeRate.get() != this.negativeRateLimit) {
      this.negativeRateLimit = this.loggedNegativeRate.get();
    }
  }

  public double calculate(double input) {
    double currentTime = MathSharedStore.getTimestamp();
    double elapsedTime = currentTime - prevTime;
    prevVal +=
        MathUtil.clamp(
            input - prevVal, negativeRateLimit * elapsedTime, positiveRateLimit * elapsedTime);
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
