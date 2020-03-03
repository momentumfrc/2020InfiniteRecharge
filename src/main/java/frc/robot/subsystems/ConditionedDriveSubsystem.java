package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConditionedDriveSubsystem extends SubsystemBase {
  private double spd_limit;
  private final double[] spd_limits = { 0.25, 0.33333, 0.5, 0.66666, 0.75, 1 };
  private int spd_limit_index;

  public ConditionedDriveSubsystem(DriveSubsystem subsystem) {
    spd_limit_index = 0;
  }

  public void setSpeedLimit(double limit) {
    spd_limit = limit;
  }

  public void incSpeedLimit() {
    spd_limit_index += 1;
    if (spd_limit_index > 5)
      spd_limit_index = 0;
    spd_limit = spd_limits[spd_limit_index];
  }
}