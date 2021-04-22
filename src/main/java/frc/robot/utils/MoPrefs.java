package frc.robot.utils;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public final class MoPrefs {
  private static final String TABLE_NAME = "Preferences";
  private static MoPrefs instance;
  private static boolean safePrefs = false;
  private final NetworkTable m_table;

  public enum MoPrefsKey {
    INTAKE_ROLLER_SETPOINT(0.125),

    INTAKE_ROLLER_ACC_RAMP(0.1),

    CLIMBER_ENCODER_LIMIT(10),

    SHOOTER_HOOD_SETPOINT(100),

    SHOOTER_HOOD_POSITION_TOLERANCE(2),

    SHOOTER_GATE_SETPOINT(1),

    SHOOTER_PID_SETPOINT(4500),

    STORAGE_SPEED(0.75),

    SHOOTER_FLYWHEEL_TOLERANCE(100), // RPM

    // The encoder setpoint that will reliably hit the Outer Port
    // when the robot is sitting up against the Power Port.
    SHOOT_FROM_WALL_HOOD_SETPOINT(60),

    SHOOTER_KP(0.00005),

    SHOOTER_KI(0.000001),

    SHOOTER_KD(0),

    SHOOTER_KIZ(0.0000001),

    SHOOTER_KFF(0.00017),

    // PID constants for the drivetrain
    DRIVE_KP(0.00005),

    DRIVE_KI(0.000001),

    DRIVE_KD(0),

    DRIVE_KIZ(0.0000001),

    DRIVE_KFF(0.25),

    // PID constants for the shooter hood
    HOOD_KP(0.15),

    HOOD_KI(1e-6),

    HOOD_KD(0),

    HOOD_KIZ(0),

    HOOD_KFF(0), // TODO: This probably shouldn't be zero. Need to tune.

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

  public static synchronized MoPrefs getInstance() {
    if (!safePrefs) {
      throw new IllegalStateException("Do not call preferences before RobotInit()");
    }
    if (instance == null) {
      instance = new MoPrefs();
    }
    return instance;
  }

  public static void safeForPrefs() {
    safePrefs = true;
  }

  public double get(MoPrefsKey key) {
    return getEntry(key).getDouble(key.getDefaultValue());
  }

  public void init(MoPrefsKey key) {
    getEntry(key).setDefaultDouble(key.getDefaultValue());
  }

  public void set(MoPrefsKey key, double value) {
    getEntry(key).setDouble(value);
  }

  private MoPrefs() {
    m_table = NetworkTableInstance.getDefault().getTable(TABLE_NAME);
    m_table.getEntry(".type").setString("RobotPreferences");
    m_table.addEntryListener((table, key, entry, value, flags) -> entry.setPersistent(),
        EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);
  }

  public NetworkTableEntry getEntry(MoPrefsKey key) {
    return m_table.getEntry(key.getPrefsKey());
  }

}
