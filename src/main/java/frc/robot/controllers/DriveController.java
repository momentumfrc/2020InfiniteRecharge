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
   * Gets the maximum speed the robot should drive at.
   * 
   * @return the speed limit.
   */
  public double getSpeedLimiter();

  /**
   * Gets whether the robot should invert drive directions.
   * 
   * @return drive inversion boolean.
   */
  public boolean getReverseDirection();

  /**
   * Gets whether the robot should intake power cells.
   * 
   * @return whether the robot should intake power cells.
   */
  public boolean getIntakePowerCells();

  /**
   * 
   * @return Whether the pistons should fire that raise the intake back to its
   *         starting position.
   */
  public boolean getToggleIntake();

  /**
   * Gets whether the robot should shoot power cells at the lower port.
   * 
   * @return whether the robot should shoot power cells at the lower port.
   */
  public double getShootPowerCells();

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