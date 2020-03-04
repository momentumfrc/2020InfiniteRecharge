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
import edu.wpi.first.wpilibj.VictorSP;

import frc.robot.Constants;
import frc.robot.utils.MoPrefs;

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Initializes the SparkMAX motor controller, assigns it to the CAN address
   * specified, and sets it to the NEO Brushless Motor.
   */
  private final CANSparkMax shooterMAXLeft = new CANSparkMax(Constants.SPARKMAX_SHOOTER_CAN_ADDR_LEFT,
      CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax shooterMAXRight = new CANSparkMax(Constants.SPARKMAX_SHOOTER_CAN_ADDR_RIGHT,
      CANSparkMaxLowLevel.MotorType.kBrushless);
  private final VictorSP shooterGate = new VictorSP(Constants.SHOOTER_VICTORSP_PWM_CHAN);
  /**
   * The built-in PID controller provided by the Spark MAX motor controller.
   */
  private final CANPIDController shooterPIDLeft = new CANPIDController(shooterMAXLeft);
  private final CANPIDController shooterPIDRight = new CANPIDController(shooterMAXRight);
  /**
   * The target velocity of the NEO Brushless Motor.
   */
  private double shooterSetpoint = 1.0;
  /**
   * The Proportial Gain of the SparkMAX PIDF controller The weight of the
   * proportional path against the differential and integral paths is controlled
   * by this value.
   */
  private final double kP = 5e-5;
  /**
   * The Integral Gain of the SparkMAX PIDF controller The weight of the integral
   * path against the proportional and differential paths is controlled by this
   * value.
   */
  private final double kI = 1e-6;
  /**
   * The Differential Gain of the SparkMAX PIDF controller. The weight of the
   * differential path against the proportional and integral paths is controlled
   * by this value.
   */
  private final double kD = 0;
  /**
   * The Integral Zone of the SparkMAX PIDF controller. The integral accumulator
   * will reset once it hits this value.
   */
  private final double kIz = 0;
  /**
   * The Feed-Forward Gain of the SparkMAX PIDF controller. The weight of the
   * feed-forward loop as compared to the PID loop is controlled by this value.
   */
  private final double kFF = 0.000156;
  /**
   * Scales the output of the SparkMAX PIDF controller.
   */
  private final double outputRange = 0.3;
  /**
   * The maximum current the motor controller is allowed to feed to the motor, in
   * amps.
   */
  private final int currentLimit = 40;

  private boolean runGate;
  private boolean reverseGate;
  private boolean runShooter;

  private double gateSetpoint;

  public ShooterSubsystem() {
    // Applies the previously-declared values to the PIDF controller.
    shooterPIDLeft.setP(kP, 0);
    shooterPIDRight.setP(kP, 0);
    shooterPIDLeft.setI(kI, 0);
    shooterPIDRight.setI(kI, 0);
    shooterPIDLeft.setD(kD, 0);
    shooterPIDRight.setD(kD, 0);
    shooterPIDLeft.setIZone(kIz, 0);
    shooterPIDRight.setIZone(kIz, 0);
    shooterPIDLeft.setFF(kFF, 0);
    shooterPIDRight.setFF(kFF, 0);
    shooterPIDLeft.setOutputRange(-outputRange, outputRange, 0);
    shooterPIDRight.setOutputRange(-outputRange, outputRange, 0);
    // Sets the shooter motor to coast so that subsequent shots don't have to rev up
    // from 0 speed.
    shooterMAXLeft.setIdleMode(IdleMode.kCoast);
    shooterMAXRight.setIdleMode(IdleMode.kCoast);
    shooterMAXLeft.setSmartCurrentLimit(currentLimit);
    shooterMAXRight.setSmartCurrentLimit(currentLimit);
    // Sets the left shooter motor to follow the right motor, and be inverted.
    shooterMAXLeft.follow(shooterMAXRight, true);
  }

  /**
   * Passes a preset velocity to the SparkMAX PIDF controller and lets it manage
   * the NEO's velocity. Intended to be called when a button is pressed.
   */
  public void shoot() {
    if (runShooter)
      shooterPIDRight.setReference(shooterSetpoint, ControlType.kVelocity);
  }

  public void runGate() {
    runGate = true;
  }

  public void stopGate() {
    runGate = false;
  }

  public void reverseGate() {
    reverseGate = !reverseGate;
  }

  public void runShooter() {
    runShooter = true;
  }

  public void stopAll() {
    runGate = false;
    shooterMAXLeft.set(0);
    shooterMAXRight.set(0);
    runShooter = false;
  }

  /**
   * Stops the shooter motor. Note: the NEO is set to Coast. Intended to be called
   * when a button is released.
   */
  public void stopShooter() {
    shooterMAXLeft.stopMotor();
    shooterMAXRight.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    gateSetpoint = 0;
    if (runGate)
      gateSetpoint = MoPrefs.getShooterGateSetpoint();
    if (reverseGate)
      gateSetpoint *= -1;
    shooterGate.set(gateSetpoint);
  }
}
