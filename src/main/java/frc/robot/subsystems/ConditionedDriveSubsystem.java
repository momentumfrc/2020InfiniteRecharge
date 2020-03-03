package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.usfirst.frc.team4999.utils.Utils;

public class ConditionedDriveSubsystem extends SubsystemBase {
  private double spd_limit;
  private final double[] spd_limits = { 0.25, 0.33333, 0.5, 0.66666, 0.75, 1 };
  private int spd_limit_index;
  private DriveSubsystem m_subsystem;

  public ConditionedDriveSubsystem(DriveSubsystem subsystem) {
    spd_limit_index = spd_limits.length - 1;
    m_subsystem = subsystem;
  }

  public void setSpeedLimit(double limit) {
    spd_limit = limit;
  }

  public void mapToLimit(double moveRequest, double turnRequest) {
    m_subsystem.drive(Utils.map(moveRequest, -1, 1, -spd_limit, spd_limit), turnRequest);
  }

  public void incSpeedLimit() {
    ++spd_limit_index;
    if (spd_limit_index >= spd_limits.length)
      spd_limit_index = spd_limits.length - 1;
    spd_limit = spd_limits[spd_limit_index];
  }

  public void decSpeedLimit() {
    --spd_limit_index;
    if (spd_limit_index > 0)
      spd_limit_index = 0;
    spd_limit = spd_limits[spd_limit_index];
  }
}