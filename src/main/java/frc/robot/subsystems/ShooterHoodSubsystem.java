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
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ShooterHoodSubsystem extends SubsystemBase {
  private final CANSparkMax hoodNEO = new SimmableCANSparkMax(Constants.SPARKMAX_SHOOTER_HOOD_CAN_ADDR,
      CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANDigitalInput hoodLimitSwitch = hoodNEO
      .getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen);
  private final CANEncoder hoodEncoder = hoodNEO.getEncoder();
  private final CANPIDController hoodPID = hoodNEO.getPIDController();

  private static final double K_P = 0.15;
  private static final double K_I = 1e-6;
  private static final double K_D = 0;
  private static final double K_IZ = 0;
  private static final double K_FF = 0;
  private static final double PID_OUTPUT_RANGE = 1.0;
  private static final double ALLOWED_ERROR = 0;

  private final double SAFE_STOW_SPEED = -0.1;

  private boolean reliableZero;

  private double minVel = 0;
  private double maxVel = 1;
  private double maxAcc = 0.1;

  private NetworkTableEntry hoodPosNumberBar;
  private NetworkTableEntry hasReliableZero;
  private NetworkTableEntry isFullyDeployed;

  public ShooterHoodSubsystem(ShuffleboardTab tab) {
    hoodPID.setP(K_P);
    hoodPID.setI(K_I);
    hoodPID.setD(K_D);
    hoodPID.setIZone(K_IZ);
    hoodPID.setFF(K_FF);
    hoodPID.setOutputRange(-PID_OUTPUT_RANGE, PID_OUTPUT_RANGE);
    hoodLimitSwitch.enableLimitSwitch(true);

    hoodPID.setSmartMotionAccelStrategy(CANPIDController.AccelStrategy.kTrapezoidal, 0);
    hoodPID.setSmartMotionMaxVelocity(maxVel, 0);
    hoodPID.setSmartMotionMaxAccel(maxAcc, 0);
    hoodPID.setSmartMotionMinOutputVelocity(minVel, 0);
    hoodPID.setSmartMotionAllowedClosedLoopError(ALLOWED_ERROR, 0);

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
  }

  public void setHoodPosition(double posRequest) {
    // Used for autonomous and vision-tied control of the shooter hood.
    hoodPID.setReference(posRequest, ControlType.kPosition, 0);
  }

  // Raises the hood to its setpoint using position PID.
  public void deployHood() {
    if (reliableZero) {
      hoodPID.setReference(MoPrefs.getShooterHoodSetpoint(), ControlType.kPosition, 0);
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
    return hoodEncoder.getPosition();
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
    return Math.abs(MoPrefs.getShooterHoodSetpoint() - getHoodPos()) < MoPrefs.getShooterHoodPositionTolerance();
  }

  public boolean isHoodReady() {
    return hasReliableZero() && isFullyDeployed();
  }

  @Override
  public void periodic() {
    if (hoodLimitSwitch.get()) {
      zeroHood();
      reliableZero = true;
    }
    // Gets the position, in rotations, from the encoder, and passes it to a
    // Shuffleboard widget
    hoodPosNumberBar.setDouble(getHoodPos());
    // Gets whether the hood has touched the limit switch, and passes it to a
    // Shuffleboard widget
    hasReliableZero.setBoolean(hasReliableZero());
    // Gets whether the hood is within an acceptable distance of the setpoint, and
    // passes it to a Shuffleboard widget
    isFullyDeployed.setBoolean(isFullyDeployed());
  }

  // Resets the encoder to 0.
  private void zeroHood() {
    hoodEncoder.setPosition(0);
  }
}
