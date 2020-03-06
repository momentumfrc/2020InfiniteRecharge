package frc.robot.utils;

import org.usfirst.frc.team4999.utils.MoPDP.OvercurrentMonitor;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;

public class SafeSP extends VictorSP {

  private static double DEFAULT_COOLDOWN_SPEED = 0;
  private static int DEFAULT_COOLDOWN_TIME = 1000;

  private enum State {
    kNormal, kOvercurrent, kCooldown
  }

  private State m_state = State.kNormal;
  private long then;

  private OvercurrentMonitor monitor;
  private int cooldownTime;
  private double cooldownSpeed;

  public SafeSP(int pwmChannel, double cooldownSpeed, int cooldownTime, OvercurrentMonitor monitor) {
    super(pwmChannel);
    this.monitor = monitor;

    this.cooldownSpeed = Math.abs(cooldownSpeed);
    this.cooldownTime = cooldownTime;
  }

  public SafeSP(int pwmChannel, OvercurrentMonitor monitor) {
    this(pwmChannel, DEFAULT_COOLDOWN_SPEED, DEFAULT_COOLDOWN_TIME, monitor);
  }

  @Override
  public void set(double value) {
    switch (m_state) {
    case kNormal:
      if (monitor.check()) {
        m_state = State.kOvercurrent;
        super.set(Utils.clip(value, -cooldownSpeed, cooldownSpeed));
      } else {
        super.set(value);
      }
      break;
    case kOvercurrent:
      if (!monitor.check()) {
        m_state = State.kCooldown;
        then = getTimeMillis();
      }
      super.set(Utils.clip(value, -cooldownSpeed, cooldownSpeed));
      break;
    case kCooldown:
      long now = getTimeMillis();
      if (now - then > cooldownTime) {
        m_state = State.kNormal;
        set(value);
      } else {
        super.set(Utils.clip(value, -cooldownSpeed, cooldownSpeed));
      }
      break;
    }
  }

  private long getTimeMillis() {
    return (long) (Timer.getFPGATimestamp() * 1000L);
  }
}