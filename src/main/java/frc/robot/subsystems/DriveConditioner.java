package frc.robot.subsystems;

import org.usfirst.frc.team4999.utils.Utils;

public class DriveConditioner {
  private double spd_limit;
  private final double[] spd_limits = { 0.25, 0.33333, 0.5, 0.66666, 0.75, 1 };
  private int spd_limit_index;

  public DriveConditioner() {
    spd_limit_index = spd_limits.length - 1;
  }

  public void setSpeedLimit(double limit) {
    spd_limit = limit;
  }

  public double conditionMove(double moveRequest) {
    return Utils.map(moveRequest, -1, 1, -spd_limit, spd_limit);
  }

  public double conditionTurn(double turnRequest) {
    return turnRequest;
  }

  public void incSpeedLimit() {
    ++spd_limit_index;
    if (spd_limit_index >= spd_limits.length)
      spd_limit_index = spd_limits.length - 1;
    spd_limit = spd_limits[spd_limit_index];
  }

  public void decSpeedLimit() {
    spd_limit_index = Math.max(spd_limit_index - 1, 0);
    spd_limit = spd_limits[spd_limit_index];
  }

  public void stop() {

  }
}