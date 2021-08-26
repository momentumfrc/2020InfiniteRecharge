package frc.robot.controllers;

public interface DriveController {
  /**
   * Gets the current speed the robot should move at in arcade drive.
   * 
   * @return the requested move speed.
   */
  public double getMoveRequest();

  /**
   * Gets the current rotation the robot should turn to in arcade drive.
   * 
   * @return the requested turn magnitude.
   */
  public double getTurnRequest();

  /**
   * Gets the current speed the left wheels of the robot should move at in tank
   * drive.
   * 
   * @return the requested move speed.
   */
  public double getLeftStick();

  /**
   * Gets the current speed the right wheels of the robot should move at in tank
   * drive.
   * 
   * @return the requested move speed.
   */
  public double getRightStick();
}