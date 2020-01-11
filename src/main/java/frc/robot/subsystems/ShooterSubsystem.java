/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Initializes the SparkMAX motor controller, assigns it to the CAN address
   * specified, and sets it to the NEO Brushless Motor.
   */
  private final CANSparkMax ShooterMAX = new CANSparkMax(Constants.SPARKMAX_SHOOTER_CAN_ADDR,
      CANSparkMaxLowLevel.MotorType.kBrushless);
  /**
   * The built-in PID controller provided by the Spark MAX motor controller.
   */
  private final CANPIDController ShooterPID = new CANPIDController(ShooterMAX);
  /**
   * The target velocity of the NEO Brushless Motor.
   */
  private final double ShooterSetpoint = 1.0;
  /**
   * The Proportial Gain of the SparkMAX PIDF controller The weight of the
   * proportional path against the differential and integral paths is controlled
   * by this value.
   */
  private final double kP = 1;
  /**
   * The Integral Gain of the SparkMAX PIDF controller The weight of the integral
   * path against the proportional and differential paths is controlled by this
   * value.
   */
  private final double kI = 1;
  /**
   * The Differential Gain of the SparkMAX PIDF controller. The weight of the
   * differential path against the proportional and integral paths is controlled
   * by this value.
   */
  private final double kD = 1;
  /**
   * The Integral Zone of the SparkMAX PIDF controller. The integral accumulator
   * will reset once it hits this value.
   */
  private final double kIz = 1;
  /**
   * The Feed-Forward Gain of the SparkMAX PIDF controller. The weight of the
   * feed-forward loop as compared to the PID loop is controlled by this value.
   */
  private final double kFF = 1;
  /**
   * Scales the output of the SparkMAX PIDF controller.
   */
  private final double OutputRange = 1;

  public ShooterSubsystem() {
    // Applies the previously-declared values to the PIDF controller.
    ShooterPID.setP(kP, 0);
    ShooterPID.setI(kI, 0);
    ShooterPID.setD(kD, 0);
    ShooterPID.setIZone(kIz, 0);
    ShooterPID.setFF(kFF, 0);
    ShooterPID.setOutputRange(-OutputRange, OutputRange, 0);
    // Sets the shooter motor to coast so that subsequent shots don't have to rev up
    // from 0 speed.
    ShooterMAX.setIdleMode(IdleMode.kCoast);
  }

  /**
   * Passes a preset velocity to the SparkMAX PIDF controller and lets it manage
   * the NEO's velocity. Intended to be called when a button is pressed.
   */
  public void shoot() {
    ShooterPID.setReference(ShooterSetpoint, ControlType.kVelocity);
  }

  /**
   * Stops the shooter motor. Note: the NEO is set to Coast. Intended to be called
   * when a button is released.
   */
  public void stopShooter() {
    ShooterMAX.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
