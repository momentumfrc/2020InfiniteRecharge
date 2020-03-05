package frc.robot.controllers;

public interface DriveController {
  /**
   * Gets the current speed the robot should move at.
   * 
   * @return the requested move speed.
   */
  public double getMoveRequest();

  /**
   * Gets the current rotation the robot should turn to.
   * 
   * @return the requested turn magnitude.
   */
  public double getTurnRequest();
}