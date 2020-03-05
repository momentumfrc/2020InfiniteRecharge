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

public class ShooterHoodSubsystem extends SubsystemBase {
  private final CANSparkMax hoodNEO = new CANSparkMax(Constants.SPARKMAX_SHOOTER_HOOD_CAN_ADDR,
      CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANDigitalInput hoodLimitSwitch = hoodNEO
      .getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen);
  private final CANEncoder hoodEncoder = hoodNEO.getEncoder();
  private final CANPIDController hoodPID = hoodNEO.getPIDController();

  private final double kP = 5e-5;
  private final double kI = 1e-6;
  private final double kD = 0;
  private final double kIz = 0;
  private final double kFF = 0.000156;
  private final double kMaxOutput = 0.3;
  private final double kMinOutput = -0.3;
  private final double allowedErr = 0;

  private double hoodPos;
  public boolean isDeployed = false;
  private boolean deploy = false;
  private boolean reliableZero;

  private double minVel = 0;
  private double maxVel = 1;
  private double maxAcc = 0.1;

  public ShooterHoodSubsystem() {
    hoodPID.setP(kP);
    hoodPID.setI(kI);
    hoodPID.setD(kD);
    hoodPID.setIZone(kIz);
    hoodPID.setFF(kFF);
    hoodPID.setOutputRange(kMinOutput, kMaxOutput);
    hoodLimitSwitch.enableLimitSwitch(true);

    hoodPID.setSmartMotionAccelStrategy(CANPIDController.AccelStrategy.kTrapezoidal, 0);
    hoodPID.setSmartMotionMaxVelocity(maxVel, 0);
    hoodPID.setSmartMotionMaxAccel(maxAcc, 0);
    hoodPID.setSmartMotionMinOutputVelocity(minVel, 0);
    hoodPID.setSmartMotionAllowedClosedLoopError(allowedErr, 0);

    hoodNEO.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen).enableLimitSwitch(false);

    hoodNEO.setIdleMode(IdleMode.kBrake);

    reliableZero = false;
    stopHood();
  }

  public void moveHood(double posRequest) {
    // Used for autonomous and vision-tied control of the shooter hood.
    hoodPID.setReference(posRequest, ControlType.kSmartMotion, 0);
  }

  public void deployHood() {
    deploy = true;
  }

  public void stowHood() {
    deploy = false;
  }

  public double getHoodPos() {
    return hoodPos;
  }

  public void stopHood() {
    hoodNEO.set(0);
  }

  public boolean getFullyDeployed() {
    boolean isFullyDeployed = false;
    if (hoodEncoder.getPosition() >= MoPrefs.getShooterHoodFullyDeployedPos())
      isFullyDeployed = true;
    return isFullyDeployed;
  }

  @Override
  public void periodic() {
    hoodPos = hoodEncoder.getPosition();
    if (hoodLimitSwitch.get()) {
      zeroHood();
      reliableZero = true;
    }

    double hoodSetpoint = 0;

    if (deploy && reliableZero)
      hoodSetpoint = MoPrefs.getShooterHoodSetpoint();

    hoodPID.setReference(hoodSetpoint, ControlType.kSmartMotion, 0);
  }

  private void zeroHood() {
    setHoodPos(0);
  }

  private void setHoodPos(int newPos) {
    hoodEncoder.setPosition(newPos);
  }
}
