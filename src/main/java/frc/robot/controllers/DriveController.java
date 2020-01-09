package frc.robot.controllers;

public interface DriveController {
  /**
   * Gets the current speed the robot should move at.
   * 
   * @return the requested move speed.
   */
  abstract public double getMoveRequest();

  /**
   * Gets the current rotation the robot should turn to.
   * 
   * @return the requested turn magnitude.
   */
  abstract public double getTurnRequest();

  /**
   * Gets the maximum speed the robot should drive at.
   * 
   * @return the speed limit.
   */
  abstract public double getSpeedLimiter();

  /**
   * Gets whether the robot should invert drive directions.
   * 
   * @return drive inversion boolean.
   */
  abstract public boolean getReverseDirection();

  /**
   * Gets whether the robot should intake power cells.
   * 
   * @return whether the robot should intake power cells.
   */
  abstract public boolean getIntakePowerCells();

  /**
   * Gets whether the robot should shoot power cells at the lower port.
   * 
   * @return whether the robot should shoot power cells at the lower port.
   */
  abstract public boolean getShootPowerCellsLow();

  /**
   * Gets whether the robot should shoot power cells at the outer port.
   * 
   * @return whether the robot should shoot power cells at the outer port.
   */
  abstract public boolean getShootPowerCellsOuter();

  /**
   * Gets whether the robot should shoot power cells at the inner port.
   * 
   * @return whether the robot should shoot power cells at the inner port.
   */
  abstract public boolean getShootPowerCellsInner();

  /**
   * Gets whether the robot should attempt the Rotation Control objective.
   * 
   * @return whether the robot should attempt the Rotation Control objective.
   */
  abstract public boolean getRotationCtrl();

  /**
   * Gets whether the robot should attempt the Position Control objective.
   * 
   * @return whether the robot should attempt the Position Control objective.
   */
  abstract public boolean getPositionCtrl();

  /**
   * Gets whether the robot should compact itself to fit under the TRENCH.
   * 
   * @return whether the robot should compact itself to fit under the TRENCH.
   */
  abstract public boolean getCompactUnderTrench();
}