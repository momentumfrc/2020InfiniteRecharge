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

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
  private static final double K_P = 5e-5;
  /**
   * The Integral Gain of the SparkMAX PIDF controller The weight of the integral
   * path against the proportional and differential paths is controlled by this
   * value.
   */
  private static final double K_I = 1e-6;
  /**
   * The Differential Gain of the SparkMAX PIDF controller. The weight of the
   * differential path against the proportional and integral paths is controlled
   * by this value.
   */
  private static final double K_D = 0;
  /**
   * The Integral Zone of the SparkMAX PIDF controller. The integral accumulator
   * will reset once it hits this value.
   */
  private static final double K_IZ = 0;
  /**
   * The Feed-Forward Gain of the SparkMAX PIDF controller. The weight of the
   * feed-forward loop as compared to the PID loop is controlled by this value.
   */
  private static final double K_FF = 0.000156;
  /**
   * Scales the output of the SparkMAX PIDF controller.
   */
  private static final double PID_OUTPUT_RANGE = 1;
  /**
   * The maximum current the motor controller is allowed to feed to the motor, in
   * amps.
   */
  private static final int CURRENT_LIMIT = 40;

  private boolean enablePID = true;
  private final ShuffleboardTab tab;

  private final ShooterHoodSubsystem shooterHood;

  public ShooterSubsystem(final ShooterHoodSubsystem shooterHood, ShuffleboardTab tab) {

    this.shooterHood = shooterHood;

    // Applies the previously-declared values to the PIDF controller.
    shooterPIDLeft.setP(K_P, 0);
    shooterPIDRight.setP(K_P, 0);
    shooterPIDLeft.setI(K_I, 0);
    shooterPIDRight.setI(K_I, 0);
    shooterPIDLeft.setD(K_D, 0);
    shooterPIDRight.setD(K_D, 0);
    shooterPIDLeft.setIZone(K_IZ, 0);
    shooterPIDRight.setIZone(K_IZ, 0);
    shooterPIDLeft.setFF(K_FF, 0);
    shooterPIDRight.setFF(K_FF, 0);

    shooterPIDLeft.setOutputRange(-PID_OUTPUT_RANGE, PID_OUTPUT_RANGE, 0);
    shooterPIDRight.setOutputRange(-PID_OUTPUT_RANGE, PID_OUTPUT_RANGE, 0);
    // Sets the shooter motor to coast so that subsequent shots don't have to rev up
    // from 0 speed.
    follower_shooterMAXLeft.setIdleMode(IdleMode.kCoast);
    leader_shooterMAXRight.setIdleMode(IdleMode.kCoast);
    follower_shooterMAXLeft.setSmartCurrentLimit(CURRENT_LIMIT);
    leader_shooterMAXRight.setSmartCurrentLimit(CURRENT_LIMIT);

    // The shooter should idle and run in the positive direction for normal
    // operation.
    // Flip this invert setting if it runs backwards.
    leader_shooterMAXRight.setInverted(false);

    // Sets the left shooter motor to follow the right motor, and be inverted.
    follower_shooterMAXLeft.follow(leader_shooterMAXRight, true);

    this.tab = tab;

    NetworkTableEntry shooterPIDchooser = tab.add("Shooter PID", true).withWidget(BuiltInWidgets.kToggleSwitch)
        .getEntry();
    shooterPIDchooser.addListener(notice -> enablePID = notice.value.getBoolean(),
        EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
  }

  public void shoot() {
    // fast shooter wheel
    // deploy hood
    // run gate if both of "" are good

    // 5500 RPM is the approximate max free speed of the shooter flywheel.
    double openLoopSetpoint = Utils.map(MoPrefs.getShooterPIDSetpoint(), -5500, 5500, -1, 1);
    if (enablePID) {
      shooterPIDRight.setReference(MoPrefs.getShooterPIDSetpoint(), ControlType.kVelocity);
    } else {
      leader_shooterMAXRight.set(openLoopSetpoint);
    }

    shooterHood.deployHood();

    final boolean shooterHoodReady = shooterHood.hasReliableZero() && shooterHood.getFullyDeployed();
    final boolean shooterWheelReady = Math.abs(MoPrefs.getShooterPIDSetpoint() - shooterEncoder.getVelocity()) < MoPrefs
        .getShooterFlywheelTolerance();
    if (shooterHoodReady && shooterWheelReady) {
      shooterGate.set(MoPrefs.getShooterGateSetpoint());
    } else {
      shooterGate.set(0);
    }
  }

  public void shootFromWall() {
    // fast flywheel
    // deploy hood at lower angle
    // run gate if both of "" are good
    double openLoopSetpoint = Utils.map(MoPrefs.getShooterPIDSetpoint(), -5500, 5500, -1, 1);
    if (enablePID) {
      shooterPIDRight.setReference(MoPrefs.getShooterPIDSetpoint(), ControlType.kVelocity);
    } else {
      leader_shooterMAXRight.set(openLoopSetpoint);
    }

    shooterHood.setHoodPosition(MoPrefs.getShootFromWallHoodSetpoint());

    final boolean shooterHoodReady = shooterHood.hasReliableZero() && shooterHood.getFullyDeployed();
    final boolean shooterWheelReady = Math.abs(MoPrefs.getShooterPIDSetpoint() - shooterEncoder.getVelocity()) < MoPrefs
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
    leader_shooterMAXRight.stopMotor();
  }

  public void purge() {
    // stow hood
    // reverse gate
    // reverse shooter wheel
    leader_shooterMAXRight.set(-0.2);
    shooterGate.set(-1 * MoPrefs.getShooterGateSetpoint());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Flywheel Speed", shooterEncoder.getVelocity());
    SmartDashboard.putNumber("Flywheel Position", shooterEncoder.getPosition());
  }
}
