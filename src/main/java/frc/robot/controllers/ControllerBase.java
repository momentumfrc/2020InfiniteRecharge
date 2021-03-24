package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;

import org.usfirst.frc.team4999.controllers.LogitechF310;

public class ControllerBase implements DriveController {
  protected final XboxController xbox;
  protected final LogitechF310 f310;

  public ControllerBase(XboxController xbox, LogitechF310 f310) {
    this.xbox = xbox;
    this.f310 = f310;
  }

  public double getMoveRequest() {
    double moveRequest = f310.getRawAxis(0); // Inverted, because +Y is down on most game controllers.
    return moveRequest;
  }

  public double getTurnRequest() {
    double turnRequest = -f310.getRawAxis(1);
    return turnRequest;
  }

  public boolean getIncHoodPos() {
    return xbox.getPOV() == 180;
  }

  public boolean getDecHoodPos() {
    return xbox.getPOV() == 0;
  }
}