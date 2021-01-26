/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Map;

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
  private double kP = 5e-5;
  /**
   * The Integral Gain of the SparkMAX PIDF controller The weight of the integral
   * path against the proportional and differential paths is controlled by this
   * value.
   */
  private double kI = 1e-6;
  /**
   * The Differential Gain of the SparkMAX PIDF controller. The weight of the
   * differential path against the proportional and integral paths is controlled
   * by this value.
   */
  private double kD = 0;
  /**
   * The Integral Zone of the SparkMAX PIDF controller. The integral accumulator
   * will reset once it hits this value.
   */
  private double kIZ = 0;
  /**
   * The Feed-Forward Gain of the SparkMAX PIDF controller. The weight of the
   * feed-forward loop as compared to the PID loop is controlled by this value.
   */
  private double kFF = 0.000156;
  /**
   * Scales the output of the SparkMAX PIDF controller.
   */
  private static final double PID_OUTPUT_RANGE = 1;
  /**
   * The maximum current the motor controller is allowed to feed to the motor, in
   * amps.
   */
  private static final int CURRENT_LIMIT = 40;
  /**
   * The (rough) maximum free speed of the shooter flywheel, in RPM.
   */
  private static final int MAX_FREE_SPEED = 5500;

  private double setpoint;

  private boolean enablePID = true;

  private final ShooterHoodSubsystem shooterHood;

  private NetworkTableEntry flywheelSpeed;
  private NetworkTableEntry isFlywheelReady;
  private NetworkTableEntry shooterSetpoint;
  private NetworkTableEntry kPSlider;
  private NetworkTableEntry kISlider;
  private NetworkTableEntry kIZSlider;
  private NetworkTableEntry kDSlider;
  private NetworkTableEntry kFFSlider;

  public ShooterSubsystem(final ShooterHoodSubsystem shooterHood, ShuffleboardTab tab) {

    this.shooterHood = shooterHood;

    // Applies the previously-declared values to the PIDF controller.
    shooterPIDLeft.setP(kP, 0);
    shooterPIDRight.setP(kP, 0);
    shooterPIDLeft.setI(kI, 0);
    shooterPIDRight.setI(kI, 0);
    shooterPIDLeft.setD(kD, 0);
    shooterPIDRight.setD(kD, 0);
    shooterPIDLeft.setIZone(kIZ, 0);
    shooterPIDRight.setIZone(kIZ, 0);
    shooterPIDLeft.setFF(kFF, 0);
    shooterPIDRight.setFF(kFF, 0);

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

    NetworkTableEntry shooterPIDchooser = tab.add("Shooter PID", true).withWidget(BuiltInWidgets.kToggleSwitch)
        .getEntry();

    shooterPIDchooser.addListener(notice -> enablePID = notice.value.getBoolean(),
        EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    // Configures a Shuffleboard widget for flywheel speed. It will display as a
    // read-only dial with a minimum value of -6000, a maximum of 6000, and the
    // speed displayed as text below.
    flywheelSpeed = tab.add("Flywheel Speed (RPM)", 0).withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("Min", -6000, "Max", 6000, "Show value", true)).getEntry();

    // Adds a Shuffleboard widget to show whether the flywheel is spinning within a
    // certain tolerance of the setpoint. See isFlywheelReady().
    isFlywheelReady = tab.add("Is Flywheel Ready?", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();

    shooterSetpoint = tab.addPersistent("Shooter Setpoint", 4500).withWidget(BuiltInWidgets.kTextView).getEntry();
    shooterSetpoint.addListener(notice -> setpoint = notice.value.getDouble(),
        EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    kPSlider = tab.addPersistent("kP", 5e-5).withWidget(BuiltInWidgets.kTextView).getEntry();
    kPSlider.addListener(notice -> kP = notice.value.getDouble(), EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    kISlider = tab.addPersistent("kI", 1e-6).withWidget(BuiltInWidgets.kTextView).getEntry();
    kISlider.addListener(notice -> kI = notice.value.getDouble(), EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    kIZSlider = tab.addPersistent("kIZ", 1e-6).withWidget(BuiltInWidgets.kTextView).getEntry();
    kIZSlider.addListener(notice -> kIZ = notice.value.getDouble(),
        EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    kDSlider = tab.addPersistent("kD", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
    kDSlider.addListener(notice -> kD = notice.value.getDouble(), EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    kFFSlider = tab.addPersistent("kFF", 0.000156).withWidget(BuiltInWidgets.kTextView).getEntry();
    kFFSlider.addListener(notice -> kFF = notice.value.getDouble(),
        EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
  }

  public void shoot(double hoodSetpoint) {
    // fast shooter wheel
    // deploy hood
    // run gate if both of "" are good

    double pidSetpoint = setpoint;

    if (enablePID) {
      shooterPIDRight.setReference(pidSetpoint, ControlType.kVelocity);
    } else {
      leader_shooterMAXRight.set(getOpenLoopSetpoint(pidSetpoint));
    }

    shooterHood.setHoodPosition(hoodSetpoint);

    if (shooterHood.isHoodReady() && isFlywheelReady()) {
      shooterGate.set(MoPrefs.getShooterGateSetpoint());
    } else {
      shooterGate.set(0);
    }
  }

  public void idle() {
    // stop gate
    // stop shooter wheel
    shooterGate.stopMotor();
    leader_shooterMAXRight.stopMotor();
  }

  public void purge() {
    // reverse gate
    // reverse shooter wheel
    leader_shooterMAXRight.set(-0.2);
    shooterGate.set(-1 * MoPrefs.getShooterGateSetpoint());
  }

  /**
   * 
   * @param closedLoopSetpoint The PID setpoint, in RPM
   * @return A number between -1 and 1 to be used for openloop motor control
   */
  private double getOpenLoopSetpoint(double closedLoopSetpoint) {
    return Utils.map(closedLoopSetpoint, -MAX_FREE_SPEED, MAX_FREE_SPEED, -1, 1);
  }

  private boolean isFlywheelReady() {
    return Math.abs(MoPrefs.getShooterPIDSetpoint() - shooterEncoder.getVelocity()) < MoPrefs
        .getShooterFlywheelTolerance();
  }

  @Override
  public void periodic() {
    flywheelSpeed.setDouble(shooterEncoder.getVelocity());
    isFlywheelReady.setBoolean(isFlywheelReady());
  }
}
