/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.usfirst.frc.team4999.utils.MoPDP;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Constants;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.MoUtils;
import frc.robot.utils.SafeSP;

public class IntakeSubsystem extends SubsystemBase {

  private final double SAFE_SPEED = 0;
  private final int SAFE_COOLDOWN_MS = 1000;
  private final double UNSAFE_CURRENT_LIMIT = 20; // amperes
  private final int UNSAFE_CURRENT_TIMEOUT_MS = 1000;

  private final VictorSP intakeSP;
  private final VictorSP intakeSP2;

  private final DoubleSolenoid intakePiston = new DoubleSolenoid(Constants.INTAKE_PISTON_PCM_CHAN_DEPLOY,
      Constants.INTAKE_PISTON_PCM_CHAN_STOW);

  public boolean isLowered = false;
  private double lastPower;

  private final DoubleSolenoid.Value deploy = DoubleSolenoid.Value.kForward;
  private final DoubleSolenoid.Value stow = DoubleSolenoid.Value.kReverse;

  public IntakeSubsystem(MoPDP pdp) {
    intakeSP = new SafeSP(Constants.INTAKE_VICTORSP_PWM_CHAN, SAFE_SPEED, SAFE_COOLDOWN_MS, pdp
        .MakeOvercurrentMonitor(Constants.INTAKE_VICTORSP_PDP_CHAN, UNSAFE_CURRENT_LIMIT, UNSAFE_CURRENT_TIMEOUT_MS));
    intakeSP2 = new SafeSP(Constants.INTAKE_VICTORSP_PWM_CHAN_2, SAFE_SPEED, SAFE_COOLDOWN_MS, pdp
        .MakeOvercurrentMonitor(Constants.INTAKE_VICTORSP_PDP_CHAN, UNSAFE_CURRENT_LIMIT, UNSAFE_CURRENT_TIMEOUT_MS));

    intakeSP.setInverted(false);
    intakeSP2.setInverted(true);

    raiseIntake();
  }

  public void idle() {
    setMotorsWithoutRamp(0);
  }

  public void runIntakeFwd() {
    setMotorsWithRamp(MoPrefs.getIntakeRollerSetpoint());
  }

  public void runIntakeRvs() {
    setMotorsWithRamp(-1 * MoPrefs.getIntakeRollerSetpoint());
  }

  private void setMotorsWithRamp(double power) {
    double currPower = MoUtils.rampMotor(power, lastPower, MoPrefs.getIntakeRollerAccRamp());
    lastPower = currPower;
    intakeSP.set(currPower);
    intakeSP2.set(currPower);
  }

  private void setMotorsWithoutRamp(double power) {
    lastPower = power;
    intakeSP.set(power);
    intakeSP2.set(power);
  }

  public void toggleIntakeDeploy() {
    if (isLowered)
      raiseIntake();
    else
      lowerIntake();
  }

  public void raiseIntake() {
    intakePiston.set(stow);
    isLowered = false;
  }

  public void lowerIntake() {
    intakePiston.set(deploy);
    isLowered = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
