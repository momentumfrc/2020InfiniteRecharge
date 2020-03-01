package frc.robot.utils;

import edu.wpi.first.wpilibj.Preferences;

public class MoPrefs {
  public static final double INTAKE_ROLLER_SETPOINT = 0.3;
  private static boolean safePrefs = false;

  public static void safeForPrefs() {
    safePrefs = true;
  }

  private static Preferences getPrefs() {
    if (safePrefs)
      return Preferences.getInstance();
    throw new RuntimeException("Do not call preferences before RobotInit()");
  }

  public static double getDouble(String key, double def) {
    Preferences prefs = getPrefs();
    if (!prefs.containsKey(key)) {
      prefs.putDouble(key, def);
      System.out.format("Prefs default key=%s value=%f\n", key, def);
    }
    double value = prefs.getDouble(key, def);
    return value;
  }

  public static void setDouble(String key, double value) {
    Preferences prefs = getPrefs();
    prefs.putDouble(key, value);
    System.out.format("set pref: %s=%f\n", key, value);
  }

  public static double getIntakeRollerSetpoint() {
    return getDouble("INTAKE_ROLLER_SETPOINT", INTAKE_ROLLER_SETPOINT);
  }
}
