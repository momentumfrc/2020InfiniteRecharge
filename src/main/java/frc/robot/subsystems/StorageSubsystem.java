package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.VictorSP;

public class StorageSubsystem extends SubsystemBase {
  private final VictorSP storage = new VictorSP(Constants.STORAGE_VICTORSP_PWM_CHAN);
  
  
  public StorageSubsystem() {
  
  }
  
  public void runStorage() {
    
  }
  
  public void stopStorage() {
    
  }
  
  public void purge() {
    
  }
  
  @Override
  public void periodic() {
    
  }
}
