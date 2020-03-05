package frc.robot.subsystems.conditioners;

public class ComposedConditioner implements DriveConditioner {

  private final DriveConditioner[] conditioners;

  public ComposedConditioner(DriveConditioner... conditioners) {
    this.conditioners = conditioners;
  }

  @Override
  public double conditionMove(double moveRequest) {
    double value = moveRequest;
    for (int i = 0; i < conditioners.length; i++) {
      value = conditioners[i].conditionMove(value);
    }
    return value;
  }

  @Override
  public double conditionTurn(double turnRequest) {
    double value = turnRequest;
    for (int i = 0; i < conditioners.length; i++) {
      value = conditioners[i].conditionTurn(value);
    }
    return value;
  }

}