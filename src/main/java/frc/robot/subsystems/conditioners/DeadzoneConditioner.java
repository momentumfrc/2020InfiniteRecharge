package frc.robot.subsystems.conditioners;

import org.usfirst.frc.team4999.utils.Utils;

public class DeadzoneConditioner implements DriveConditioner {
  private final double move_request_deadzone = 0.1;
  private final double turn_request_deadzone = 0.1;

  @Override
  public double conditionMove(double moveRequest) {
    return Utils.deadzone(moveRequest, move_request_deadzone);
  }

  @Override
  public double conditionTurn(double turnRequest) {
    return Utils.deadzone(turnRequest, turn_request_deadzone);
  }
}