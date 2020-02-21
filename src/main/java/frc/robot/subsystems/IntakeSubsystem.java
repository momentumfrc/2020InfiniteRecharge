/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final VictorSP intakeSPrt = new VictorSP(Constants.INTAKE_VICTORSP_PWM_CHAN_LF);
  private final VictorSP intakeSPlf = new VictorSP(Constants.INTAKE_VICTORSP_PWM_CHAN_RT);

  public IntakeSubsystem() {
    intakeSPrt.setInverted(true);
  }

  public void runIntake() {
    intakeSPrt.set(0.3);
    intakeSPlf.set(0.3);
  }

  public void stopIntake() {
    intakeSPrt.stopMotor();
    intakeSPlf.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
