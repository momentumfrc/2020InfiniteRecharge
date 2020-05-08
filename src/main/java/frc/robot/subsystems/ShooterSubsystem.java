/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import org.usfirst.frc.team4999.utils.Utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.SimmableCANSparkMax;

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Initializes the SparkMAX motor controller, assigns it to the CAN address
   * specified, and sets it to the NEO Brushless Motor.
   */
  private final CANSparkMax follower_shooterMAXLeft = new SimmableCANSparkMax(Constants.SPARKMAX_SHOOTER_CAN_ADDR_LEFT,
      CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax leader_shooterMAXRight = new SimmableCANSparkMax(Constants.SPARKMAX_SHOOTER_CAN_ADDR_RIGHT,
      CANSparkMaxLowLevel.MotorType.kBrushless);
  private final VictorSP shooterGate = new VictorSP(Constants.SHOOTER_VICTORSP_PWM_CHAN);
  /**
   * The built-in PID controller provided by the Spark MAX motor controller.
   */
  private final CANPIDController shooterPIDLeft = follower_shooterMAXLeft.getPIDController();
  private final CANPIDController shooterPIDRight = leader_shooterMAXRight.getPIDController();
  private final CANEncoder shooterEncoder = leader_shooterMAXRight.getEncoder();
  /**
   * The Proportial Gain of the SparkMAX PIDF controller The weight of the
   * proportional path against the differential and integral paths is controlled
   * by this value.
   */
  private final double kP = 5e-5;// 0.107;
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
  private final double outputRange = 1;
  /**
   * The maximum current the motor controller is allowed to feed to the motor, in
   * amps.
   */
  private final int currentLimit = 40;

  private final boolean enablePID = true;

  private final boolean maintainFlywheelAtIdle = false;

  private final ShooterHoodSubsystem shooterHood;

  public ShooterSubsystem(final ShooterHoodSubsystem shooterHood) {

    this.shooterHood = shooterHood;

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
    follower_shooterMAXLeft.setIdleMode(IdleMode.kCoast);
    leader_shooterMAXRight.setIdleMode(IdleMode.kCoast);
    follower_shooterMAXLeft.setSmartCurrentLimit(currentLimit);
    leader_shooterMAXRight.setSmartCurrentLimit(currentLimit);

    // The shooter should idle and run in the positive direction for normal
    // operation.
    // Flip this invert setting if it runs backwards.
    leader_shooterMAXRight.setInverted(false);

    // Sets the left shooter motor to follow the right motor, and be inverted.
    follower_shooterMAXLeft.follow(leader_shooterMAXRight, true);
  }

  public void shoot() {
    // fast shooter wheel
    // run gate if both of "" are good
    double dumbSetpoint = Utils.map(MoPrefs.getShooterPIDSetpoint(), -5500, 5500, -1, 1);
    if (enablePID) {
      shooterPIDRight.setReference(MoPrefs.getShooterPIDSetpoint(), ControlType.kVelocity);
    } else {
      leader_shooterMAXRight.set(dumbSetpoint);
    }

    final boolean shooterHoodReady = shooterHood.hasReliableZero() && shooterHood.getFullyDeployed();
    final boolean shooterWheelReady = Math.abs(MoPrefs.getShooterFlywheelSetpoint() - getCurrVelocity()) < MoPrefs
        .getShooterFlywheelTolerance();
    if (shooterHoodReady && shooterWheelReady) {
      shooterGate.set(MoPrefs.getShooterGateSetpoint());
    } else {
      shooterGate.set(0);
    }

  }

  public void idle() {
    // stop gate
    // slow shooter wheel
    shooterGate.stopMotor();

    if (maintainFlywheelAtIdle) {
      if (enablePID) {
        shooterPIDRight.setReference(MoPrefs.getShooterFlywheelIdle(), ControlType.kVelocity);
      } else {
        leader_shooterMAXRight.set(MoPrefs.getShooterFlywheelIdle());
      }
    } else {
      leader_shooterMAXRight.stopMotor();
    }
  }

  public void purge() {
    // stow hood
    // reverse gate
    // reverse shooter wheel
    leader_shooterMAXRight.set(-MoPrefs.getShooterFlywheelIdle());
    shooterGate.set(-MoPrefs.getShooterGateSetpoint());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Flywheel Speed", shooterEncoder.getVelocity());
    SmartDashboard.putNumber("Flywheel Position", shooterEncoder.getPosition());
  }
}
