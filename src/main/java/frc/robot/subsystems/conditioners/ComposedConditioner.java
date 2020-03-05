package frc.robot.subsystems.conditioners;

public class ComposedConditioner implements DriveConditioner {

  private final DriveConditioner[] conditioners;

  public ComposedConditioner(DriveConditioner... conditioners) {
    this.conditioners = conditioners;
  }

  @Override
  public double conditionMove(double moveRequest) {
    double value = moveRequest;
    for (DriveConditioner cond : conditioners) {
      value = cond.conditionMove(value);
    }
    return value;
  }

  @Override
  public double conditionTurn(double turnRequest) {
    double value = turnRequest;
    for (DriveConditioner cond : conditioners) {
      value = cond.conditionTurn(value);
    }
    return value;
  }

}