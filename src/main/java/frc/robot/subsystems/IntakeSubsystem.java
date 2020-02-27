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

  private final DoubleSolenoid intakePistonL = new DoubleSolenoid(Constants.INTAKE_PISTON_PCM_CHAN_LF_DEPLOY,
      Constants.INTAKE_PISTON_PCM_CHAN_LF_STOW);
  private final DoubleSolenoid intakePistonR = new DoubleSolenoid(Constants.INTAKE_PISTON_PCM_CHAN_RT_DEPLOY,
      Constants.INTAKE_PISTON_PCM_CHAN_RT_STOW);

  public boolean isLowered = false;
  private boolean runIntake = false;
  private boolean reverseIntake = false;
  private final double rollerSetpoint = 0.3;

  private final DoubleSolenoid.Value deploy = DoubleSolenoid.Value.kForward;
  private final DoubleSolenoid.Value stow = DoubleSolenoid.Value.kReverse;

  public IntakeSubsystem() {
    intakeSPrt.setInverted(true);
  }

  public void runIntake(boolean run, boolean reverse) {
    runIntake = run;
    reverseIntake = reverse;
  }

  public void reverseIntake() {
    reverseIntake = !reverseIntake;
  }

  public void toggleIntakeDeploy() {
    if (isLowered)
      raiseIntake();
    else
      lowerIntake();
  }

  public void raiseIntake() {
    intakePistonL.set(deploy);
    intakePistonR.set(deploy);
    isLowered = false;
  }

  public void lowerIntake() {
    intakePistonL.set(stow);
    intakePistonR.set(stow);
    isLowered = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (runIntake && !reverseIntake) {
      intakeSPrt.set(rollerSetpoint);
      intakeSPlf.set(rollerSetpoint);
    } else if (runIntake && reverseIntake) {
      intakeSPlf.set(-rollerSetpoint);
      intakeSPrt.set(-rollerSetpoint);
    } else {
      intakeSPlf.set(0);
      intakeSPrt.set(0);
    }
  }
}
