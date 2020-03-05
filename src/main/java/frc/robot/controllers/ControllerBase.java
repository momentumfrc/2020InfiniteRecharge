package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import org.usfirst.frc.team4999.controllers.LogitechF310;
import org.usfirst.frc.team4999.utils.Utils;

public class ControllerBase implements DriveController {
  protected final XboxController xbox;
  protected final LogitechF310 f310;

  static final double DEADZONE = 0.1;
  static final double MOVE_CURVE = 2;
  static final double TURN_CURVE = 2;

  static final double[] SPEEDS = { .125, .25, .375, .5, .625, .875, 1 };
  int currentSpeed = SPEEDS.length - 1;

  public ControllerBase(XboxController xbox, LogitechF310 f310) {
    this.xbox = xbox;
    this.f310 = f310;
  }

  public double getMoveRequest() {
    double moveRequest = -xbox.getY(XboxController.Hand.kLeft);
    return Utils.deadzone(moveRequest, DEADZONE);
  }

  public double getTurnRequest() {
    double turnRequest = xbox.getX(XboxController.Hand.kRight);
    return Utils.deadzone(turnRequest, DEADZONE);
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

  public boolean getToggleIntake() {
    return false;
  }

  public boolean getIntakePowerCells() {
    return f310.getBumperPressed(Hand.kLeft);
  }

  public double getShootPowerCells() {
    return xbox.getTriggerAxis(Hand.kRight);
  }

  public boolean getPurgePowerCells() {
    return xbox.getBumper(Hand.kLeft);
  }

  public boolean getShootPowerCellsOuter() {
    return f310.getBumperPressed(Hand.kRight);
  }

  public boolean getShootPowerCellsInner() {
    return false;
  }

  public boolean getRotationCtrl() {
    return f310.getAButtonPressed();
  }

  public boolean getPositionCtrl() {
    return f310.getBButtonPressed();
  }

  public boolean getCompactUnderTrench() {
    return f310.getYButtonPressed();
  }
}