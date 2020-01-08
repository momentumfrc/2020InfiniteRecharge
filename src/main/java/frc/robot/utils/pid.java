package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;

public class pid {

  private double kP, kI, kD, kF, iErrZone;

  private double totalErr;
  private double lastErr;
  private double lastTime;

  public pid() {

  }

  public double calculate(double target, double current) {
    // calculate time for dT
    double now = Timer.getFPGATimestamp() * 1000.0; // FPGA time is in seconds with microsecond resolution
    double dTime = now - lastTime;
    lastTime = now;

    // Calculate error
    double err = target - current;

    // Calculate totalErr
    if (Math.abs(err) > iErrZone) {
      totalErr = 0;
    } else {
      totalErr += err * dTime;
    }

    // Calculate dErr
    double dErr = dTime > 0 ? (err - lastErr) / dTime : 0.0;
    lastErr = err;

    // Combine all the parts
    return kF * target + kP * err + kI * totalErr + kD * dErr;
  }
}