package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import org.usfirst.frc.team4999.controllers.LogitechF310;
import org.usfirst.frc.team4999.utils.Utils;

public class ControllerBase implements DriveController {
  protected final XboxController xbox;
  protected final LogitechF310 f310;

  public ControllerBase(XboxController xbox, LogitechF310 f310) {
    this.xbox = xbox;
    this.f310 = f310;
  }

  public double getMoveRequest() {
    double moveRequest = -xbox.getY(XboxController.Hand.kLeft);
    return moveRequest;
  }

  public double getShootPowerCells() {
    return Utils.deadzone(xbox.getTriggerAxis(Hand.kRight), 0.2);
  }

  public double getTurnRequest() {
    double turnRequest = xbox.getX(XboxController.Hand.kRight);
    return turnRequest;
  }

  public boolean getIncHoodPos() {
    return xbox.getPOV() == 180;
  }

  public boolean getDecHoodPos() {
    return xbox.getPOV() == 0;
  }
}