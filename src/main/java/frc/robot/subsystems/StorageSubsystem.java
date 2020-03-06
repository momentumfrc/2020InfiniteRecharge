package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.usfirst.frc.team4999.utils.MoPDP;

import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.Constants;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.SafeSP;

public class StorageSubsystem extends SubsystemBase {

  private final double SAFE_SPEED = 0;
  private final int SAFE_COOLDOWN_MS = 1000;
  private final double UNSAFE_CURRENT_LIMIT = 30; // amperes
  private final int UNSAFE_CURRENT_TIMEOUT_MS = 1000;

  private final VictorSP storage;
  private double speed;

  public StorageSubsystem(MoPDP pdp) {
    storage = new SafeSP(Constants.STORAGE_VICTORSP_PWM_CHAN, SAFE_SPEED, SAFE_COOLDOWN_MS, pdp
        .MakeOvercurrentMonitor(Constants.STORAGE_VICTORSP_PDP_CHAN, UNSAFE_CURRENT_LIMIT, UNSAFE_CURRENT_TIMEOUT_MS));
    stop();
  }

  public void run() {
    speed = MoPrefs.getStorageSpeed();
  }

  public void reverse() {
    speed = -MoPrefs.getStorageSpeed();
  }

  public void stop() {
    speed = 0;
  }

  @Override
  public void periodic() {
    storage.set(speed);
  }
}
