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

  /**
   * Gets whether the robot should attempt the Rotation Control objective.
   * 
   * @return whether the robot should attempt the Rotation Control objective.
   */
  public boolean getRotationCtrl();

  /**
   * Gets whether the robot should attempt the Position Control objective.
   * 
   * @return whether the robot should attempt the Position Control objective.
   */
  public boolean getPositionCtrl();

  /**
   * Gets whether the robot should compact itself to fit under the TRENCH.
   * 
   * @return whether the robot should compact itself to fit under the TRENCH.
   */
  public boolean getCompactUnderTrench();
}