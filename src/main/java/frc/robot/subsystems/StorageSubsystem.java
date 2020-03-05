package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.Constants;
import frc.robot.utils.MoPrefs;

public class StorageSubsystem extends SubsystemBase {
  private final VictorSP storage;
  private double speed;

  public StorageSubsystem() {
    storage = new VictorSP(Constants.STORAGE_VICTORSP_PWM_CHAN);
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
