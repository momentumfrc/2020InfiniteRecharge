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

import edu.wpi.first.networktables.NetworkTableInstance;
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
  /**
   * The Proportial Gain of the SparkMAX PIDF controller The weight of the
   * proportional path against the differential and integral paths is controlled
   * by this value.
   */
  private final double kP = 5e-2;
  /**
   * The Integral Gain of the SparkMAX PIDF controller The weight of the integral
   * path against the proportional and differential paths is controlled by this
   * value.
   */
  private final double kI = 0;
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
  private final double kFF = 0.00156;
  /**
   * Scales the output of the SparkMAX PIDF controller.
   */
  private final double outputRange = 0.3;
  /**
   * The maximum current the motor controller is allowed to feed to the motor, in
   * amps.
   */
  private final int currentLimit = 40;

  private final boolean enablePID = true;

  private final boolean maintainFlywheelAtIdle = false;

  private ShooterHoodSubsystem shooterHood;

  public ShooterSubsystem(ShooterHoodSubsystem shooterHood) {

    this.shooterHood = shooterHood;

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);

    // Applies the previously-declared values to the PIDF controller.
    shooterPIDLeft.setP(kP, 0);
    shooterPIDRight.setP(kP, 0);
    shooterPIDLeft.setI(kI, 0);
    shooterPIDRight.setI(kI, 0);
    shooterPIDLeft.setD(kD, 0);
    shooterPIDRight.setD(kD, 0);
    shooterPIDLeft.setIZone(kIz, 0);
    shooterPIDRight.setIZone(kIz, 0);

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
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);

    if (enablePID) {
      shooterPIDRight.setReference(MoPrefs.getShooterFlywheelSetpoint(), ControlType.kVelocity);
    } else {
      leader_shooterMAXRight.set(MoPrefs.getShooterFlywheelSetpoint());
    }
    if (shooterHood.hasReliableZero() && shooterHood.getFullyDeployed() && MoPrefs.getShooterFlywheelSetpoint()
        - leader_shooterMAXRight.getEncoder().getVelocity() < MoPrefs.getShooterFlywheelTolerance()) {
      shooterGate.set(MoPrefs.getShooterGateSetpoint());
    } else {
      shooterGate.set(0);
    }

  }

  public void shootAuto() {
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    shooterHood.deployHood();

    leader_shooterMAXRight.set(MoPrefs.getShooterFlywheelSetpoint());

    if (shooterHood.hasReliableZero() && shooterHood.getFullyDeployed() && MoPrefs.getShooterFlywheelSetpoint()
        - leader_shooterMAXRight.getEncoder().getVelocity() < MoPrefs.getShooterFlywheelTolerance()) {
      shooterGate.set(MoPrefs.getShooterGateSetpoint());
    } else {
      shooterGate.set(0);
    }
  }

  public void idle() {
    // stop gate
    // slow shooter wheel
    shooterGate.stopMotor();
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);

    if (maintainFlywheelAtIdle) {
      if (enablePID) {
        shooterPIDRight.setReference(MoPrefs.getShooterFlywheelIdle(), ControlType.kVelocity);
      } else {
        leader_shooterMAXRight.set(MoPrefs.getShooterFlywheelIdle());
      }
    } else {
      if (enablePID) {
        shooterPIDRight.setReference(0, ControlType.kVelocity);
      } else {
        leader_shooterMAXRight.stopMotor();
      }
    }
  }

  public void purge() {
    // stow hood
    // reverse gate
    // reverse shooter wheel
    if (enablePID) {
      shooterPIDRight.setReference(-1 * MoPrefs.getShooterFlywheelIdle(), ControlType.kVelocity);
    } else {
      leader_shooterMAXRight.set(-1 * MoPrefs.getShooterFlywheelIdle());
    }

    shooterGate.set(-1 * MoPrefs.getShooterGateSetpoint());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Flywheel Speed", leader_shooterMAXRight.getEncoder().getVelocity());
  }
}
