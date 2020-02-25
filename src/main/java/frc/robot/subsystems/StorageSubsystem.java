package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.Constants;

public class StorageSubsystem extends SubsystemBase {
  private final VictorSP StorageGate;
  private final VictorSP StorageMid;

  private boolean runMid = false;
  private boolean runGate = false;

  public StorageSubsystem() {
    StorageMid = new VictorSP(Constants.STORAGE_MID_PWM_CHAN);
    StorageGate = new VictorSP(Constants.STORAGE_GATE_PWM_CHAN);
  }

  public void runMid(boolean run) {
    runMid = run;
  }

  public void runGate(boolean run) {
    runGate = run;
  }

  public void stopAll() {
    runMid = false;
    runGate = false;
  }

  @Override
  public void periodic() {
    if (runMid)
      StorageMid.set(Constants.STORAGE_MID_VAL);
    if (runGate)
      StorageGate.set(Constants.STORAGE_GATE_VAL);
  }
}