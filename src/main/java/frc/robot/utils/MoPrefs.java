package frc.robot.utils;

import edu.wpi.first.wpilibj.Preferences;

public class MoPrefs {
  static final double INTAKE_ROLLER_SETPOINT = 1.0;
  /**
   * The maximum accleration of the intake roller, in motor setpoint per scheduler
   * cycle.
   */
  static final double INTAKE_ROLLER_ACC_RAMP = 0.1;
  static final int CLIMBER_ENCODER_LIMIT = 10000;
  static final double SHOOTER_HOOD_SETPOINT = 15;
  static final double STORAGE_SETPOINT = 1.0;

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

  public static double getClimberEncoderLimit() {
    return getDouble("CLIMBER_ENCODER_LIMIT", CLIMBER_ENCODER_LIMIT);
  }

  public static double getShooterHoodSetpoint() {
    return getDouble("SHOOTER_HOOD_SETPOINT", SHOOTER_HOOD_SETPOINT);
  }

  public static double getStorageSpeed() {
    return getDouble("STORAGE_SPEED_SETPOINT", STORAGE_SETPOINT);
  }

  public static double getIntakeRollerAccRamp() {
    return getDouble("INTAKE_ROLLER_ACC_RAMP", INTAKE_ROLLER_ACC_RAMP);
  }
}
