package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
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

  private double minVel;
  private double maxVel;
  private double maxAcc;

  public ShooterHoodSubsystem() {
    hoodPID.setP(kP);
    hoodPID.setI(kI);
    hoodPID.setD(kD);
    hoodPID.setIZone(kIz);
    hoodPID.setFF(kFF);
    hoodPID.setOutputRange(kMinOutput, kMaxOutput);
    hoodLimitSwitch.enableLimitSwitch(true);
  }

  public void moveHood(double moveRequest) {
    // Used for autonomous and vision-tied control of the shooter hood.
  }

  public void raiseHood() {

  }

  public void lowerHood() {

  }

  public void getHoodPos() {

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
