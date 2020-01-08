package frc.robot.utils;

public class Utils {
  public static double clip(double value, double min, double max) {
    return Math.min(Math.max(value, min), max);
  }
}