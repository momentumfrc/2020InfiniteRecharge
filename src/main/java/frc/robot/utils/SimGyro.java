package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;

public class SimGyro {
  private double angle = 0; // degrees
  private double rate = 0;
  private double lastAngle = 0;
  private double lastTime = 0;

  public SimGyro() {

  }

  public double getAngle() {
    return angle;
  }

  public double getRate() {
    return rate;
  }

  public void reset() {
    angle = rate = lastAngle = 0;
  }

  public void update(double newAngle) {
    rate = (newAngle - lastAngle) / (Timer.getFPGATimestamp() - lastTime);
    lastAngle = angle;
    lastTime = Timer.getFPGATimestamp();
    angle = newAngle;
  }
}