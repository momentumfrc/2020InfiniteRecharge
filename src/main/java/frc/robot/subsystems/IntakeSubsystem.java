/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final VictorSP intakeSPrt = new VictorSP(Constants.INTAKE_VICTORSP_PWM_CHAN_LF);
  private final VictorSP intakeSPlf = new VictorSP(Constants.INTAKE_VICTORSP_PWM_CHAN_RT);

  private final DoubleSolenoid intakePistonL = new DoubleSolenoid(Constants.INTAKE_PISTON_PCM_CHAN_LF_F,
      Constants.INTAKE_PISTON_PCM_CHAN_LF_R);
  private final DoubleSolenoid intakePistonR = new DoubleSolenoid(Constants.INTAKE_PISTON_PCM_CHAN_RT_F,
      Constants.INTAKE_PISTON_PCM_CHAN_RT_R);

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

  public void raiseIntake() {
    intakePistonL.set(DoubleSolenoid.Value.kForward);
    intakePistonR.set(DoubleSolenoid.Value.kForward);
  }

  public void lowerIntake() {
    intakePistonL.set(DoubleSolenoid.Value.kReverse);
    intakePistonR.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
