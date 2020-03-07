package frc.robot.utils;

import org.usfirst.frc.team4999.utils.Utils;

public class MoUtils {

  public static double rampMotor(double desiredValue, double lastValue, double maxDelta) {
    double delta = desiredValue - lastValue;
    double abs_max_delta = Math.abs(maxDelta);
    double clamped_delta = Utils.clip(delta, -abs_max_delta, abs_max_delta);
    return lastValue + clamped_delta;
  }
}