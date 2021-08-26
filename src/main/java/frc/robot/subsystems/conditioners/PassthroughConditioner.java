package frc.robot.subsystems.conditioners;

public class PassthroughConditioner implements DriveConditioner {

  @Override
  public double conditionMove(double moveRequest) {
    return moveRequest;
  }

  @Override
  public double conditionTurn(double turnRequest) {
    return turnRequest;
  }

  @Override
  public double conditionTank(double tankRequest) {
    return tankRequest;
  }

}