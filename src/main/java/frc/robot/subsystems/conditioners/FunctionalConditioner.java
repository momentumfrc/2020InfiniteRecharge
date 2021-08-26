package frc.robot.subsystems.conditioners;

import java.util.function.DoubleUnaryOperator;

public class FunctionalConditioner implements DriveConditioner {

  private DoubleUnaryOperator moveOperator;
  private DoubleUnaryOperator turnOperator;
  private DoubleUnaryOperator tankOperator = value -> value;

  /**
   * A conditioner for use with lambdas.
   * 
   * Example: new FunctionalConditioner((mr) -> mr, (tr) -> -tr));
   * 
   * @param moveOperator MoveRequest operation
   * @param turnOperator TurnRequest operation
   */
  public FunctionalConditioner(DoubleUnaryOperator moveOperator, DoubleUnaryOperator turnOperator) {
    this.moveOperator = moveOperator;
    this.turnOperator = turnOperator;
  }

  public FunctionalConditioner(DoubleUnaryOperator moveOperator, DoubleUnaryOperator turnOperator,
      DoubleUnaryOperator tankOperator) {
    this.moveOperator = moveOperator;
    this.turnOperator = turnOperator;
    this.tankOperator = tankOperator;
  }

  @Override
  public double conditionMove(double moveRequest) {
    return moveOperator.applyAsDouble(moveRequest);
  }

  @Override
  public double conditionTurn(double turnRequest) {
    return turnOperator.applyAsDouble(turnRequest);
  }

  @Override
  public double conditionTank(double tankRequest) {
    return tankOperator.applyAsDouble(tankRequest);
  }

}