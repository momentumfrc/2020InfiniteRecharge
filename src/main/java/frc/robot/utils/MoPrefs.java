package frc.robot.utils;

import edu.wpi.first.wpilibj.Preferences;

public class MoPrefs {
  static final double INTAKE_ROLLER_SETPOINT = 0.125;
  static final double INTAKE_ROLLER_ACC_RAMP = 0.1;
  static final int CLIMBER_ENCODER_LIMIT = 10;
  static final double SHOOTER_HOOD_SETPOINT = 100;
  static final double SHOOTER_HOOD_POSITION_TOLERANCE = 2;
  static final double SHOOTER_GATE_SETPOINT = 1;
  static final double SHOOTER_PID_SETPOINT = 4500;
  static final double STORAGE_SPEED = 0.75;
  static final double SHOOTER_FLYWHEEL_TOLERANCE = 100; // RPM
  static final double SHOOT_FROM_WALL_HOOD_SETPOINT = 60; // The encoder setpoint that will reliably hit the Outer Port
                                                          // when the robot is sitting up against the Power Port.

  // PID constants for the shooter flywheel
  static final double SHOOTER_KP = 0.00005;
  static final double SHOOTER_KI = 0.000001;
  static final double SHOOTER_KD = 0;
  static final double SHOOTER_KIZ = 0.0000001;
  static final double SHOOTER_KFF = 0.00000156;

  // PID constants for the drivetrain
  static final double DRIVE_KP = 0.00005;
  static final double DRIVE_KI = 0.000001;
  static final double DRIVE_KD = 0;
  static final double DRIVE_KIZ = 0.0000001;
  static final double DRIVE_KFF = 0.00000156;

  // PID constants for the shooter hood
  static final double HOOD_KP = 0.15;
  static final double HOOD_KI = 1e-6;
  static final double HOOD_KD = 0;
  static final double HOOD_KIZ = 0;
  static final double HOOD_KFF = 0;
  static final double HOOD_ALLOWED_ERR = 2;
  static final double HOOD_OUT_RANGE = 1;

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
    return getDouble("SHOOTER_HOOD_POS_TOLERANCE", SHOOTER_HOOD_POSITION_TOLERANCE);
  }

  public static double getShooterGateSetpoint() {
    return getDouble("SHOOTER_GATE_SETPOINT", SHOOTER_GATE_SETPOINT);
  }

  public static double getIntakeRollerAccRamp() {
    return getDouble("INTAKE_ROLLER_ACC_RAMP", INTAKE_ROLLER_ACC_RAMP);
  }

  public static double getStorageSpeed() {
    return getDouble("STORAGE_SPEED", STORAGE_SPEED);
  }

  public static double getShooterFlywheelTolerance() {
    return getDouble("FLYWHEEL_TOLER", SHOOTER_FLYWHEEL_TOLERANCE);
  }

  public static double getShooterPIDSetpoint() {
    return getDouble("Shooter PID Setpoint", SHOOTER_PID_SETPOINT);
  }

  public static double getShootFromWallHoodSetpoint() {
    return getDouble("Shoot From Wall Hood Setpoint", SHOOT_FROM_WALL_HOOD_SETPOINT);
  }

  public static double getShooterKP() {
    return getDouble("SHOOTER_KP", SHOOTER_KP);
  }

  public static double getShooterKI() {
    return getDouble("SHOOTER_KI", SHOOTER_KI);
  }

  public static double getShooterKD() {
    return getDouble("SHOOTER_KD", SHOOTER_KD);
  }

  public static double getShooterKIZ() {
    return getDouble("SHOOTER_KIZ", SHOOTER_KIZ);
  }

  public static double getShooterKFF() {
    return getDouble("SHOOTER_KFF", SHOOTER_KFF);
  }

  public static double getDriveKP() {
    return getDouble("DRIVE_KP", DRIVE_KP);
  }

  public static double getDriveKI() {
    return getDouble("DRIVE_KI", DRIVE_KI);
  }

  public static double getDriveKD() {
    return getDouble("DRIVE_KD", DRIVE_KD);
  }

  public static double getDriveKIZ() {
    return getDouble("DRIVE_KIZ", DRIVE_KIZ);
  }

  public static double getDriveKFF() {
    return getDouble("DRIVE_KFF", DRIVE_KFF);
  }

  public static double getHoodKP() {
    return getDouble("HOOD_KP", HOOD_KP);
  }

  public static double getHoodKI() {
    return getDouble("HOOD_KI", HOOD_KI);
  }

  public static double getHoodKD() {
    return getDouble("HOOD_KD", HOOD_KD);
  }

  public static double getHoodKIZ() {
    return getDouble("HOOD_KIZ", HOOD_KIZ);
  }

  public static double getHoodKFF() {
    return getDouble("HOOD_KFF", HOOD_KFF);
  }

  public static double getHoodAllowedErr() {
    return getDouble("HOOD_ALLOWED_ERR", HOOD_ALLOWED_ERR);
  }

  public static double getHoodOutRange() {
    return getDouble("HOOD_OUT_RANGE", HOOD_OUT_RANGE);
  }
}
