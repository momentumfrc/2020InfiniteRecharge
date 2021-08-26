package frc.robot.subsystems.conditioners;

public interface DriveConditioner {
  public double conditionMove(double moveRequest);

  public double conditionTurn(double turnRequest);

  public double conditionTank(double tankRequest);
}