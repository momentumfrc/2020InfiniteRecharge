/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.MoPrefs.MoPrefsKey;

public class ClimberSubsystem extends SubsystemBase {
  private final VictorSP climberSP = new VictorSP(Constants.CLIMBER_VICTORSP_PWM_CHAN);
  private final Encoder encoder = new Encoder(Constants.CLIMBER_ENCODER_A_CHAN, Constants.CLIMBER_ENCODER_B_CHAN);
  private final DigitalInput limit = new DigitalInput(Constants.CLIMBER_LIMIT_SWITCH);

  private static final double CLIMB_STOP = 0;
  private static final double CLIMB_STOW = -1;
  private static final double CLIMB = 1;

  private boolean reliableZero;

  public ClimberSubsystem() {
    reliableZero = false;
    stop();
  }

  public boolean isStowed() {
    return limit.get();
  }

  public void stop() {
    climberSP.set(CLIMB_STOP);
  }

  public void stow() {
    if (isStowed())
      stop();
    else
      climberSP.set(CLIMB_STOW);
  }

  public void climb() {
    if (!reliableZero || encoder.get() > MoPrefs.get(MoPrefsKey.CLIMBER_ENCODER_LIMIT))
      stop();
    else
      climberSP.set(CLIMB);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (limit.get()) {
      encoder.reset();
      reliableZero = true;
    }
  }
}