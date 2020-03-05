package frc.robot.subsystems;

import org.usfirst.frc.team4999.utils.Utils;

public class DriveConditioner {
  private final double[] spd_limits = { 0.25, 0.33333, 0.5, 0.66666, 0.75, 1 };
  private int spd_limit_index;

  public DriveConditioner() {
    spd_limit_index = spd_limits.length - 1;
  }

  public double conditionMove(double moveRequest) {
    double spd_limit = spd_limits[spd_limit_index];
    return Utils.map(moveRequest, -1, 1, -spd_limit, spd_limit);
  }

  public double conditionTurn(double turnRequest) {
    return turnRequest;
  }

  public void incSpeedLimit() {
    spd_limit_index = Math.min(spd_limit_index + 1, spd_limits.length - 1);
  }

  public void decSpeedLimit() {
    spd_limit_index = Math.max(spd_limit_index - 1, 0);
  }
}