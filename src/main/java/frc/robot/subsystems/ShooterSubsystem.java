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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.SimmableCANSparkMax;
import frc.robot.utils.MoPrefs.MoPrefsKey;

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
  private static final int MAX_FREE_SPEED = 5700;

  private boolean enablePID = true;

  private final ShooterHoodSubsystem shooterHood;
  private final IntakeSubsystem intake;
  private final Limelight limelight;

  private NetworkTableEntry flywheelSpeed;
  private NetworkTableEntry isFlywheelReady;

  public ShooterSubsystem(final ShooterHoodSubsystem shooterHood, ShuffleboardTab tab, final IntakeSubsystem intake,
      final Limelight limelight) {

    this.shooterHood = shooterHood;
    this.intake = intake;
    this.limelight = limelight;

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
    // graph of the flywheel speed from the last 20 seconds.
    flywheelSpeed = tab.add("Flywheel Speed", 0).withWidget(BuiltInWidgets.kGraph).getEntry();

    // Adds a Shuffleboard widget to show whether the flywheel is spinning within a
    // certain tolerance of the setpoint. See isFlywheelReady().
    isFlywheelReady = tab.add("Is Flywheel Ready?", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();

    MoPrefs instance = MoPrefs.getInstance();

    // Attach listeners to the PID prefs for live tuning
    instance.getEntry(MoPrefsKey.SHOOTER_KP).addListener(
        notification -> shooterPIDRight.setP(notification.value.getDouble(), 0),
        EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate);
    instance.getEntry(MoPrefsKey.SHOOTER_KI).addListener(
        notification -> shooterPIDRight.setI(notification.value.getDouble(), 0),
        EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate);
    instance.getEntry(MoPrefsKey.SHOOTER_KD).addListener(
        notification -> shooterPIDRight.setD(notification.value.getDouble(), 0),
        EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate);
    instance.getEntry(MoPrefsKey.SHOOTER_KIZ).addListener(
        notification -> shooterPIDRight.setIZone(notification.value.getDouble(), 0),
        EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate);
    instance.getEntry(MoPrefsKey.SHOOTER_KFF).addListener(
        notification -> shooterPIDRight.setFF(notification.value.getDouble(), 0),
        EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate);

    // Ensure the PID prefs exist. Defaults are picked up from MoPrefsKey.
    instance.init(MoPrefsKey.SHOOTER_KP);
    instance.init(MoPrefsKey.SHOOTER_KI);
    instance.init(MoPrefsKey.SHOOTER_KD);
    instance.init(MoPrefsKey.SHOOTER_KIZ);
    instance.init(MoPrefsKey.SHOOTER_KFF);

    // Initialize the PID in the SparkMax from MoPrefs
    shooterPIDRight.setP(instance.get(MoPrefsKey.SHOOTER_KP), 0);
    shooterPIDRight.setI(instance.get(MoPrefsKey.SHOOTER_KI), 0);
    shooterPIDRight.setD(instance.get(MoPrefsKey.SHOOTER_KD), 0);
    shooterPIDRight.setIZone(instance.get(MoPrefsKey.SHOOTER_KIZ), 0);
    shooterPIDRight.setFF(instance.get(MoPrefsKey.SHOOTER_KFF), 0);
    double outRange = PID_OUTPUT_RANGE;
    shooterPIDLeft.setOutputRange(-outRange, outRange, 0);
    shooterPIDRight.setOutputRange(-outRange, outRange, 0);
    shooterPIDRight.setSmartMotionAllowedClosedLoopError(0, 0);
  }

  private double getHoodAngle(double dist) {
    return -0.0042 * Math.pow(dist - 140, 2) + 126;
  }

  public void shoot(double hoodSetpoint) {
    // Makes sure that the intake is lowered before firing.
    if (intake.isLowered) {
      // fast shooter wheel
      // deploy hood
      // run gate if both of "" are good

      double pidSetpoint = MoPrefs.getInstance().get(MoPrefsKey.SHOOTER_PID_SETPOINT);
      if (enablePID) {
        shooterPIDRight.setReference(pidSetpoint, ControlType.kVelocity);
        System.out.println("Setting flywheel to " + pidSetpoint + " RPM\n");
      } else {
        leader_shooterMAXRight.set(getOpenLoopSetpoint(pidSetpoint));
      }

      shooterHood.setHoodPosition(hoodSetpoint);
      if (shooterHood.isHoodReady() && isFlywheelReady()) {
        shooterGate.set(MoPrefs.getInstance().get(MoPrefsKey.SHOOTER_GATE_SETPOINT));

      } else {
        shooterGate.set(0);
      }
    } else {
      System.out.println("Cannot shoot with raised intake!");
      leader_shooterMAXRight.stopMotor();
      shooterGate.stopMotor();
    }
  }

  public void shoot() {
    double hoodSetpoint = getHoodAngle(limelight.getData().dist());
    shoot(hoodSetpoint);
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
    shooterGate.set(-1 * MoPrefs.getInstance().get(MoPrefsKey.SHOOTER_GATE_SETPOINT));
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
    return Math.abs(MoPrefs.getInstance().get(MoPrefsKey.SHOOTER_PID_SETPOINT) - shooterEncoder.getVelocity()) < MoPrefs
        .getInstance().get(MoPrefsKey.SHOOTER_FLYWHEEL_TOLERANCE);
  }

  @Override
  public void periodic() {
    flywheelSpeed.setDouble(shooterEncoder.getVelocity());
    isFlywheelReady.setBoolean(isFlywheelReady());
  }
}
