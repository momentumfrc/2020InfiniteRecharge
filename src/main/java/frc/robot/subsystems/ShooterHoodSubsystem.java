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

public class ShooterHoodSubsystem extends SubsystemBase {
  private CANSparkMax hoodNEO = new CANSparkMax(Constants.SPARKMAX_SHOOTER_HOOD_CAN_ADDR,
      CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANDigitalInput hoodLimitSwitch = hoodNEO
      .getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyOpen);
  private CANEncoder hoodEncoder = hoodNEO.getEncoder();
  private CANPIDController hoodPID;

  private final double kP = 5e-5;
  private final double kI = 1e-6;
  private final double kD = 0;
  private final double kIz = 0;
  private final double kFF = 0.000156;
  private final double kMaxOutput = 0.3;
  private final double kMinOutput = -0.3;
  private final double allowedErr = 0;

  private double hoodPos;
  private boolean reliableZero;
  private boolean enableLimit;
  public boolean isDeployed = false;

  private double minVel = 0;
  private double maxVel = 0;
  private double maxAcc = 0;
  private double maxHoodPos;

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
    hoodLimitSwitch.enableLimitSwitch(enableLimit);

    hoodNEO.setIdleMode(IdleMode.kCoast);
  }

  public void moveHood(double posRequest) {
    // Used for autonomous and vision-tied control of the shooter hood.
    hoodPID.setReference(posRequest, ControlType.kSmartMotion, 0);
    enableLimit = false;
  }

  public void deployHood() {
    while (hoodPos < maxHoodPos) {
      hoodPID.setReference(-0.1, ControlType.kSmartMotion, 0);
    }
    isDeployed = true;
  }

  public void stowHood() {
    while (!hoodLimitSwitch.get()) {
      hoodPID.setReference(0.1, ControlType.kSmartMotion, 0);
    }
    isDeployed = false;
  }

  public double getHoodPos() {
    return hoodPos;
  }

  public void stopHood() {
    hoodNEO.set(0);
  }

  @Override
  public void periodic() {
    hoodPos = hoodEncoder.getPosition();
    if (hoodLimitSwitch.get()) {
      zeroHood();
    }
  }

  private void zeroHood() {
    setHoodPos(0);
    reliableZero = true;
  }

  private void setHoodPos(int newPos) {
    hoodEncoder.setPosition(newPos);
  }
}
