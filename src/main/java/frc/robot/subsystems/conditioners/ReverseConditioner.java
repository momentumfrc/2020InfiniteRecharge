package frc.robot.subsystems.conditioners;

public class ReverseConditioner implements DriveConditioner {
  private boolean reversed = false;

  public boolean isReversed() {
    return reversed;
  }

  public void setReversed(boolean value) {
    reversed = value;
  }

  public void toggleReversed() {
    reversed = !reversed;
  }

  @Override
  public double conditionMove(double moveRequest) {
    return reversed ? -moveRequest : moveRequest;
  }

  @Override
  public double conditionTurn(double turnRequest) {
    return turnRequest;
  }
}