/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;

import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax ShooterMAX = new CANSparkMax(Constants.SPARKMAX_SHOOTER_CAN_ADDR,
      CANSparkMaxLowLevel.MotorType.kBrushless);

  private final CANPIDController ShooterPID = new CANPIDController(ShooterMAX);

  private final Encoder shootEncoder = new Encoder(Constants.SHOOTER_ENCODER_CHAN_A, Constants.SHOOTER_ENCODER_CHAN_B);

  private final double ShooterSetpoint = 1.0;

  private final double kP = 1;
  private final double kI = 1;
  private final double kD = 1;
  private final double kIz = 1;
  private final double kFF = 1;
  private final double OutputRange = 1;

  public ShooterSubsystem() {
    ShooterPID.setP(kP, 0);
    ShooterPID.setI(kI, 0);
    ShooterPID.setD(kD, 0);
    ShooterPID.setIZone(kIz, 0);
    ShooterPID.setFF(kFF, 0);
    ShooterPID.setOutputRange(-OutputRange, OutputRange, 0);
  }

  public void shoot() {
    ShooterPID.setReference(ShooterSetpoint, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
