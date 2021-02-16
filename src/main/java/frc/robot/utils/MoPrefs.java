package frc.robot.utils;

import edu.wpi.first.wpilibj.Preferences;

public class MoPrefs {

  public enum MoPrefsKey {
    INTAKE_ROLLER_SETPOINT(0.125),

    INTAKE_ROLLER_ACC_RAMP(0.1),

    CLIMBER_ENCODER_LIMIT(10),

    SHOOTER_HOOD_SETPOINT(100),

    // FIXME: Copy this value to the new key
    // The original key for this value was "SHOOTER_HOOD_POS_TOLERANCE"
    // The new key is "SHOOTER_HOOD_POSITION_TOLERANCE"
    SHOOTER_HOOD_POSITION_TOLERANCE(2),

    SHOOTER_GATE_SETPOINT(1),

    // FIXME: Copy this value to the new key
    // The original key for this value was "Shooter PID Setpoint"
    // The new key is "SHOOTER_PID_SETPOINT"
    SHOOTER_PID_SETPOINT(4500),

    STORAGE_SPEED(0.75),

    // FIXME: Copy this value to the new key
    // The original key for this value was "FLYWHEEL_TOLER"
    // The new key is "SHOOTER_FLYWHEEL_TOLERANCE"
    SHOOTER_FLYWHEEL_TOLERANCE(100), // RPM

    // The encoder setpoint that will reliably hit the Outer Port
    // when the robot is sitting up against the Power Port.
    //
    // FIXME: Copy this value to the new key
    // The original key for this value was "Shoot From Wall Hood Setpoint"
    // The new key is "SHOOT_FROM_WALL_HOOD_SETPOINT"
    SHOOT_FROM_WALL_HOOD_SETPOINT(60),

    SHOOTER_KP(0.00005),

    SHOOTER_KI(0.000001),

    SHOOTER_KD(0),

    SHOOTER_KIZ(0.0000001),

    SHOOTER_KFF(0.00000156),

    // PID constants for the drivetrain
    DRIVE_KP(0.00005),

    DRIVE_KI(0.000001),

    DRIVE_KD(0),

    DRIVE_KIZ(0.0000001),

    DRIVE_KFF(0.00000156),

    // PID constants for the shooter hood
    HOOD_KP(0.15),

    HOOD_KI(1e-6),

    HOOD_KD(0),

    HOOD_KIZ(0),

    HOOD_KFF(0),

    HOOD_ALLOWED_ERR(2),

    HOOD_OUT_RANGE(1);

    private double defaultValue;

    MoPrefsKey(double defaultValue) {
      this.defaultValue = defaultValue;
    }

    public String getPrefsKey() {
      return this.name();
    }

    public double getDefaultValue() {
      return defaultValue;
    }
  }

  private MoPrefs() {
    throw new IllegalStateException("MoPrefs should be static");
  }

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

  public static double get(MoPrefsKey key) {
    return getDouble(key.getPrefsKey(), key.getDefaultValue());
  }

  public static void set(MoPrefsKey key, double value) {
    setDouble(key.getPrefsKey(), value);
  }
}
