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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  private double hoodPos;
  private boolean reliableZero;

  private double minVel = 0;
  private double maxVel = 1;
  private double maxAcc = 0.1;

  private double currSetpoint;

  public ShooterHoodSubsystem() {
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
  }

  public void setHoodPosition(double posRequest) {
    // Used for autonomous and vision-tied control of the shooter hood.
    hoodPID.setReference(posRequest, ControlType.kPosition, 0);
    currSetpoint = posRequest;
  }

  public void deployHood() {
    if (reliableZero) {
      hoodPID.setReference(MoPrefs.getShooterHoodSetpoint(), ControlType.kPosition, 0);
      currSetpoint = MoPrefs.getShooterHoodSetpoint();
    } else {
      hoodNEO.set(SAFE_STOW_SPEED);
    }
  }

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

  public boolean hasReliableZero() {
    return reliableZero;
  }

  public boolean getFullyDeployed() {
    return Math.abs(getHoodPos() - currSetpoint) < MoPrefs.getShooterHoodPositionTolerance();
  }

  @Override
  public void periodic() {
    hoodPos = hoodEncoder.getPosition();
    SmartDashboard.putNumber("pose", hoodPos);
    if (hoodLimitSwitch.get()) {
      zeroHood();
      reliableZero = true;
    }
  }

  private void zeroHood() {
    setHoodPos(0);
  }

  private void setHoodPos(int newPos) {
    hoodEncoder.setPosition(newPos);
  }
}
