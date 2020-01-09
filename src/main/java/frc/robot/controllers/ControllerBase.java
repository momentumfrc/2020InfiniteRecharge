package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import static frc.robot.utils.Utils.*;

import org.usfirst.frc.team4999.controllers.LogitechF310;

public class ControllerBase implements DriveController {
  protected XboxController xbox = new XboxController(0);
  protected LogitechF310 f310 = new LogitechF310(2);

  static final double DEADZONE = 0.1;
  static final double MOVE_CURVE = 2;
  static final double TURN_CURVE = 2;

  static final double[] SPEEDS = { .125, .25, .375, .5, .625, .875, 1 };
  int currentSpeed = SPEEDS.length - 1;

  public double getMoveRequest() {
    double moveRequest = -xbox.getY(XboxController.Hand.kLeft);
    if (-DEADZONE <= moveRequest && moveRequest <= DEADZONE) {
      return 0;
    } else {
      return Math.pow(moveRequest, MOVE_CURVE);
    }
  }

  public double getTurnRequest() {
    double turnRequest = xbox.getX(XboxController.Hand.kRight);
    if (-DEADZONE <= turnRequest && turnRequest <= DEADZONE) {

      return 0;
    } else {
      return Math.pow(turnRequest, TURN_CURVE);
    }
  }

  public double getSpeedLimiter() {
    if (xbox.getYButtonPressed() && currentSpeed < SPEEDS.length - 1)
      ++currentSpeed;
    else if (xbox.getXButtonPressed() && currentSpeed > 0)
      --currentSpeed;
    return SPEEDS[currentSpeed];
  }

  public boolean getReverseDirection() {
    return xbox.getBButtonPressed();
  }

  public boolean getIntakePowerCells() {
    return f310.getBButtonPressed();
  }

  public boolean getShootPowerCellsLow() {
    return f310.getAButtonPressed();
  }

  public boolean getShootPowerCellsOuter() {
    return f310.getXButtonPressed();
  }

  public boolean getShootPowerCellsInner() {
    return f310.getYButtonPressed();
  }

  public boolean getRotationCtrl() {
    return false;
  }

  public boolean getPositionCtrl() {
    return false;
  }

  public boolean getCompactUnderTrench() {
    return false;
  }
}