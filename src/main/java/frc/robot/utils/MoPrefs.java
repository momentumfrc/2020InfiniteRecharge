package frc.robot.utils;

import edu.wpi.first.wpilibj.Preferences;

public class MoPrefs {
  static final double INTAKE_ROLLER_SETPOINT = -0.125;
  static final double INTAKE_ROLLER_ACC_RAMP = 0.1;
  static final int CLIMBER_ENCODER_LIMIT = 10;
  static final double SHOOTER_HOOD_SETPOINT = 100;
  static final double SHOOTER_HOOD_POSITION_TOLERANCE = 2;
  static final double SHOOTER_GATE_SETPOINT = 1;
  static final double SHOOTER_FLYWHEEL_SETPOINT = 1;
  static final double SHOOTER_FLYWHEEL_IDLE = 0.2;
  static final double STORAGE_SPEED = 0.75;
  static final double SHOOTER_FLYWHEEL_TOLERANCE = 100; // RPM
  static final double AUTO_CHOOSER = 0;

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

  public static double getShooterHoodPositionTolerance() {
    return 2;
  }

  public static double getShooterGateSetpoint() {
    return getDouble("SHOOTER_GATE_SETPOINT", SHOOTER_GATE_SETPOINT);
  }

  public static double getShooterFlywheelSetpoint() {
    return getDouble("SHOOTER_FLYWHEEL_SETPOINT", SHOOTER_FLYWHEEL_SETPOINT);
  }

  public static double getShooterFlywheelIdle() {
    return getDouble("SHOOTER_FLYWHEEL_IDLE", SHOOTER_FLYWHEEL_IDLE);
  }

  public static double getIntakeRollerAccRamp() {
    return 0.1;
  }

  public static double getStorageSpeed() {
    return getDouble("STORAGE_SPEED", STORAGE_SPEED);
  }

  public static double getShooterFlywheelTolerance() {
    return getDouble("FLYWHEEL_TOLER", SHOOTER_FLYWHEEL_TOLERANCE);
  }

  public static double getAutoChooser() {
    return getDouble("AUTO_CHOOSER", AUTO_CHOOSER);
  }
}
