package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.SubsystemBase;
import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.Constants;

public class StorageSubsystem extends SubsystemBase {
  private final VictorSP StorageGate;
  private final VictorSP StorageMid;

  public StorageSubsystem() {
    StorageMid = new VictorSP(Constants.STORAGE_MID_PWM_CHAN);
    StorageGate = new VictorSP(Constants.STORAGE_GATE_PWM_CHAN);
  }
  public runMid() {
    StorageMid.set(1);
  }
  public stopMotors() {
    StorageGate.stopMotors();
    StorageMid.stopMotors();
  }
}