package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

import com.revrobotics.CANDigitalInput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.utils.MoPrefs;
import frc.robot.utils.SimmableCANSparkMax;
import frc.robot.utils.MoPrefs.MoPrefsKey;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterHoodSubsystem extends SubsystemBase {
  private final CANSparkMax hoodNEO = new SimmableCANSparkMax(Constants.SPARKMAX_SHOOTER_HOOD_CAN_ADDR,
      CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANDigitalInput hoodLimitSwitch = hoodNEO
      .getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen);
  private final CANEncoder hoodEncoder = hoodNEO.getEncoder();
  private final CANPIDController hoodPID = hoodNEO.getPIDController();

  private final double SAFE_STOW_SPEED = -0.1;

  private boolean reliableZero;

  private double minVel = 0;
  private double maxVel = 1;
  private double maxAcc = 0.1;

  private NetworkTableEntry hoodPosNumberBar;
  private NetworkTableEntry hasReliableZero;
  private NetworkTableEntry isFullyDeployed;

  private double hoodPos;

  public ShooterHoodSubsystem(ShuffleboardTab tab) {
    hoodLimitSwitch.enableLimitSwitch(true);

    hoodPID.setSmartMotionAccelStrategy(CANPIDController.AccelStrategy.kTrapezoidal, 0);
    hoodPID.setSmartMotionMaxVelocity(maxVel, 0);
    hoodPID.setSmartMotionMaxAccel(maxAcc, 0);
    hoodPID.setSmartMotionMinOutputVelocity(minVel, 0);

    hoodNEO.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen).enableLimitSwitch(false);

    hoodNEO.setIdleMode(IdleMode.kBrake);
    // The Hood needs to run in the negative direction *towards* the limit switch
    // If it runs the wrong way, flip this invert setting.
    hoodNEO.setInverted(true);

    reliableZero = false;
    stopHood();

    hoodPosNumberBar = tab.add("Hood Pos", 0).withWidget(BuiltInWidgets.kNumberBar).getEntry();
    hasReliableZero = tab.add("Has reliable zero?", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    isFullyDeployed = tab.add("Hood fully deployed?", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();

    MoPrefs instance = MoPrefs.getInstance();

    // Adds network table listeners for PID constants.
    instance.getEntry(MoPrefsKey.HOOD_KP).addListener(notification -> hoodPID.setP(notification.value.getDouble()),
        EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate);
    instance.getEntry(MoPrefsKey.HOOD_KI).addListener(notification -> hoodPID.setI(notification.value.getDouble()),
        EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate);
    instance.getEntry(MoPrefsKey.HOOD_KD).addListener(notification -> hoodPID.setD(notification.value.getDouble()),
        EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate);
    instance.getEntry(MoPrefsKey.HOOD_KIZ).addListener(notification -> hoodPID.setIZone(notification.value.getDouble()),
        EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate);
    instance.getEntry(MoPrefsKey.HOOD_KFF).addListener(notification -> hoodPID.setFF(notification.value.getDouble()),
        EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate);
    instance.getEntry(MoPrefsKey.HOOD_OUT_RANGE).addListener(
        notification -> hoodPID.setOutputRange(-notification.value.getDouble(), notification.value.getDouble()),
        EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate);
    instance.getEntry(MoPrefsKey.HOOD_ALLOWED_ERR).addListener(
        notification -> hoodPID.setSmartMotionAllowedClosedLoopError(notification.value.getDouble(), 0),
        EntryListenerFlags.kUpdate | EntryListenerFlags.kImmediate);

    instance.init(MoPrefsKey.HOOD_KP, 0);
    instance.init(MoPrefsKey.HOOD_KI, 0);
    instance.init(MoPrefsKey.HOOD_KD, 0);
    instance.init(MoPrefsKey.HOOD_KIZ, 0);
    instance.init(MoPrefsKey.HOOD_KFF, 0);
    instance.init(MoPrefsKey.HOOD_OUT_RANGE, 0);
    instance.init(MoPrefsKey.HOOD_ALLOWED_ERR, 0);

    hoodPID.setP(instance.get(MoPrefsKey.HOOD_KP), 0);
    hoodPID.setI(instance.get(MoPrefsKey.HOOD_KI), 0);
    hoodPID.setD(instance.get(MoPrefsKey.HOOD_KD), 0);
    hoodPID.setIZone(instance.get(MoPrefsKey.HOOD_KIZ), 0);
    hoodPID.setFF(instance.get(MoPrefsKey.HOOD_KFF), 0);
    double outRange = instance.get(MoPrefsKey.HOOD_OUT_RANGE);
    hoodPID.setOutputRange(-outRange, outRange, 0);
    hoodPID.setSmartMotionAllowedClosedLoopError(instance.get(MoPrefsKey.HOOD_ALLOWED_ERR), 0);
  }

  public void setHoodPosition(double posRequest) {
    // Used for autonomous and vision-tied control of the shooter hood.
    hoodPID.setReference(posRequest, ControlType.kPosition, 0);
  }

  // Raises the hood to its setpoint using position PID.
  public void deployHood() {
    if (reliableZero) {
      hoodPID.setReference(MoPrefs.getInstance().get(MoPrefsKey.SHOOTER_HOOD_SETPOINT), ControlType.kPosition, 0);
    } else {
      hoodNEO.set(SAFE_STOW_SPEED);
    }
  }

  // Lowers the shooter hood until it hits the limit switch.
  public void stowHood() {
    if (reliableZero)
      hoodPID.setReference(0, ControlType.kPosition, 0);
    else
      hoodNEO.set(SAFE_STOW_SPEED);
  }

  public double getHoodPos() {
    return hoodPos;
  }

  public void stopHood() {
    hoodNEO.set(0);
  }

  private boolean hasReliableZero() {
    // If the hood has hit the limit switch, and therefore has been zeroed, return
    // true. Otherwise, return false.
    return reliableZero;
  }

  private boolean isFullyDeployed() {
    // If the current position is in within +-positionTolerance of the setpoint,
    // return true
    // Otherwise, return false
    return Math.abs(MoPrefs.getInstance().get(MoPrefsKey.SHOOTER_HOOD_SETPOINT) - getHoodPos()) < MoPrefs.getInstance()
        .get(MoPrefsKey.SHOOTER_HOOD_POSITION_TOLERANCE);
  }

  public boolean isHoodReady() {
    return hasReliableZero() && isFullyDeployed();
  }

  @Override
  public void periodic() {
    // If the hood hits the limit switch, reset the encoder and let everything else
    // know that the zero is reliable
    if (hoodLimitSwitch.get()) {
      zeroHood();
      reliableZero = true;
    }
    // Updates the recorded position periodically so that things like isHoodReady()
    // can access it without polling the encoder too many times.
    hoodPos = hoodEncoder.getPosition();
    // Gets the position, in rotations, from the encoder, and passes it to a
    // Shuffleboard widget
    hoodPosNumberBar.setDouble(getHoodPos());
    // Gets whether the hood has touched the limit switch, and passes it to a
    // Shuffleboard widget
    hasReliableZero.setBoolean(hasReliableZero());
    // Gets whether the hood is within an acceptable distance of the setpoint, and
    // passes it to a Shuffleboard widget
    isFullyDeployed.setBoolean(isFullyDeployed());

    SmartDashboard.putNumber("Hood setpoint", hoodNEO.get());
  }

  // Resets the encoder to 0.
  private void zeroHood() {
    hoodEncoder.setPosition(0);
  }
}
