package frc.robot.choosers;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.MoPrefs.MoPrefsKey;

public class ShooterModeHandler {
  NetworkTableEntry shooterModePref;
  Boolean isSmartShooting = false;

  public ShooterModeHandler() {
    shooterModePref = MoPrefs.getInstance().getEntry(MoPrefsKey.USE_SMART_SHOOTING);
    shooterModePref.addListener(notice -> isSmartShooting = notice.value.getDouble() == 1 ? true : false,
        EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
  }

  public boolean get() {
    return isSmartShooting;
  }

  public void toggle() {
    MoPrefs.getInstance().set(MoPrefsKey.USE_SMART_SHOOTING, !isSmartShooting == true ? 1 : 0);
  }
}
