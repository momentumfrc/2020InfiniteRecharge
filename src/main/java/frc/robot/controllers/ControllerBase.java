package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import org.usfirst.frc.team4999.controllers.LogitechF310;

public class ControllerBase implements DriveController {
  protected final XboxController xbox;
  protected final LogitechF310 f310;

  public ControllerBase(XboxController xbox, LogitechF310 f310) {
    this.xbox = xbox;
    this.f310 = f310;
  }

  // These are for arcade drive
  public double getMoveRequest() {
    return -f310.getY(Hand.kLeft); // Inverted, because +Y is down on most game controllers.
  }

  public double getTurnRequest() {
    return f310.getX(Hand.kRight);
  }

  // These are for tank drive
  public double getLeftStick() {
    return -f310.getY(Hand.kLeft); // See previous comment.
  }

  public double getRightStick() {
    return -f310.getY(Hand.kRight);
  }

  public boolean getIncHoodPos() {
    return xbox.getPOV() == 180;
  }

  public boolean getDecHoodPos() {
    return xbox.getPOV() == 0;
  }
}