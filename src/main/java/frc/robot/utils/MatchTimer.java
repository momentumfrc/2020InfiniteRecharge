package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;

public class MatchTimer {
  public boolean getAfterTime(double afterTime) {
    if (afterTime < Timer.getMatchTime()) {
      return true;
    } else
      return false;
  }
}