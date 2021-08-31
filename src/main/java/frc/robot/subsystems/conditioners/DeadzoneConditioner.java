package frc.robot.subsystems.conditioners;

import org.usfirst.frc.team4999.utils.Utils;

public class DeadzoneConditioner implements DriveConditioner {
  private static final double DEFAULT_MOVE_REQUEST_DEADZONE = 0.1;
  private static final double DEFAULT_TURN_REQUEST_DEADZONE = 0.1;

  private final double move_request_deadzone;
  private final double turn_request_deadzone;

  public DeadzoneConditioner() {
    move_request_deadzone = DEFAULT_MOVE_REQUEST_DEADZONE;
    turn_request_deadzone = DEFAULT_TURN_REQUEST_DEADZONE;
  }

  public DeadzoneConditioner(double moveRequestDeadzone, double turnRequestDeadzone) {
    move_request_deadzone = moveRequestDeadzone;
    turn_request_deadzone = turnRequestDeadzone;
  }

  @Override
  public double conditionMove(double moveRequest) {
    return Utils.deadzone(moveRequest, move_request_deadzone);
  }

  @Override
  public double conditionTurn(double turnRequest) {
    return Utils.deadzone(turnRequest, turn_request_deadzone);
  }
}