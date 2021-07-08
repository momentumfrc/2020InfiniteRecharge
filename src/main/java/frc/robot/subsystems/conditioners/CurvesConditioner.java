package frc.robot.subsystems.conditioners;

import org.usfirst.frc.team4999.utils.Utils;

public class CurvesConditioner implements DriveConditioner {
  private final double move_curve = 2;
  private final double turn_curve = 3;

  @Override
  public double conditionMove(double moveRequest) {
    return Utils.curve(moveRequest, move_curve);
  }

  @Override
  public double conditionTurn(double turnRequest) {
    return Utils.curve(turnRequest, turn_curve);
  }

}