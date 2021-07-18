package frc.robot.subsystems.conditioners;

import org.usfirst.frc.team4999.utils.Utils;

public class CurvesConditioner implements DriveConditioner {
  private static final double DEFAULT_MOVE_CURVE = 2;
  private static final double DEFAULT_TURN_CURVE = 3;
  private final double move_curve;
  private final double turn_curve;

  public CurvesConditioner() {
    move_curve = DEFAULT_MOVE_CURVE;
    turn_curve = DEFAULT_TURN_CURVE;
  }

  public CurvesConditioner(double move_curve, double turn_curve) {
    this.move_curve = move_curve;
    this.turn_curve = turn_curve;
  }

  @Override
  public double conditionMove(double moveRequest) {
    return Utils.curve(moveRequest, move_curve);
  }

  @Override
  public double conditionTurn(double turnRequest) {
    return Utils.curve(turnRequest, turn_curve);
  }

}