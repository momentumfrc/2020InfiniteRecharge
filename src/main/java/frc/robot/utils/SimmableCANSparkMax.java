package frc.robot.utils;

import java.util.Arrays;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimEnum;

public class SimmableCANSparkMax extends CANSparkMax {

  private SimDevice m_simDevice;
  private SimDouble m_simSpeed;
  private SimBoolean m_simInvert;

  private SimBoolean m_fwdLimitSwitch;
  private SimBoolean m_rvsLimitSwitch;

  private SimDouble m_encoderCount;
  private SimDouble m_encoderVelocity;

  private SimEnum m_controlType;
  private final String[] controlTypes = Arrays.stream(ControlType.values())
      .sorted((firstType, secondType) -> Integer.compare(firstType.value, secondType.value)).map((type) -> type.name())
      .toArray(String[]::new);

  private class SimmableCANPIDController extends CANPIDController {

    SimmableCANPIDController(SimmableCANSparkMax max) {
      super(max);
    }

    @Override
    public CANError setReference(double value, ControlType ctrl, int pidSlot, double arbFeedforward) {
      if (m_simSpeed != null && m_controlType != null) {
        m_simSpeed.set(value);
        m_controlType.set(ctrl.value);
      }
      return super.setReference(value, ctrl, pidSlot, arbFeedforward);
    }

    @Override
    public CANError setReference(double value, ControlType ctrl, int pidSlot, double arbFeedforward,
        ArbFFUnits arbFFUnits) {
      if (m_simSpeed != null && m_controlType != null) {
        m_simSpeed.set(value);
        m_controlType.set(ctrl.value);
      }
      return super.setReference(value, ctrl, pidSlot, arbFeedforward, arbFFUnits);
    }

  }

  private class SimmableCANDigitalInput extends CANDigitalInput {
    private LimitSwitch type;
    private SimBoolean getter;

    public SimmableCANDigitalInput(CANSparkMax device, LimitSwitch limitSwitch, LimitSwitchPolarity polarity) {
      super(device, limitSwitch, polarity);

      type = limitSwitch;
    }

    @Override
    public CANError enableLimitSwitch(boolean enable) {
      if (m_simDevice != null && enable) {
        if (type == LimitSwitch.kForward) {
          if (m_fwdLimitSwitch == null) {
            m_fwdLimitSwitch = m_simDevice.createBoolean("FWD Limit Switch", false, false);
          }
          getter = m_fwdLimitSwitch;
        } else {
          if (m_rvsLimitSwitch == null) {
            m_rvsLimitSwitch = m_simDevice.createBoolean("RVS Limit Switch", false, false);
          }
          getter = m_rvsLimitSwitch;
        }
      }

      return super.enableLimitSwitch(enable);
    }

    @Override
    public boolean get() {
      if (getter != null) {
        return getter.get();
      } else {
        return super.get();
      }
    }

  }

  private class SimmableCANEncoder extends CANEncoder {

    public SimmableCANEncoder(CANSparkMax device, EncoderType sensorType, int counts_per_rev) {
      super(device, sensorType, counts_per_rev);
    }

    @Override
    public double getPosition() {
      if (m_simDevice != null) {
        if (m_encoderCount == null) {
          m_encoderCount = m_simDevice.createDouble("Encoder Pos", false, 0);
        }
        return m_encoderCount.get();
      } else {
        return super.getPosition();
      }
    }

    @Override
    public double getVelocity() {
      if (m_simDevice != null) {
        if (m_encoderVelocity == null) {
          m_encoderVelocity = m_simDevice.createDouble("Encoder Vel", false, 0);
        }
        return m_encoderVelocity.get();
      } else {
        return super.getVelocity();
      }
    }

  }

  public SimmableCANSparkMax(int deviceID, MotorType type) {
    super(deviceID, type);

    for (String s : controlTypes)
      System.out.format("%s, ", s);
    System.out.println();

    m_simDevice = SimDevice.create("Spark Max", deviceID);
    if (m_simDevice != null) {
      m_controlType = m_simDevice.createEnum("Control Mode", true, controlTypes, 0);
      m_simSpeed = m_simDevice.createDouble("Motor Output", true, 0.0);
      m_simInvert = m_simDevice.createBoolean("Inverted?", true, false);
    }
  }

  @Override
  public void set(double speed) {
    super.set(speed);

    if (m_simSpeed != null && m_simInvert != null) {
      m_simSpeed.set(speed * (m_simInvert.get() ? -1 : 1));
    }
  }

  @Override
  public void stopMotor() {
    super.stopMotor();

    if (m_simSpeed != null) {
      m_simSpeed.set(0);
    }
  }

  @Override
  public void setInverted(boolean isInverted) {
    super.setInverted(isInverted);

    if (m_simInvert != null) {
      m_simInvert.set(isInverted);
    }
  }

  @Override
  public CANPIDController getPIDController() {
    return new SimmableCANPIDController(this);
  }

  @Override
  public CANDigitalInput getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity polarity) {
    return new SimmableCANDigitalInput(this, CANDigitalInput.LimitSwitch.kForward, polarity);
  }

  @Override
  public CANDigitalInput getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity polarity) {
    return new SimmableCANDigitalInput(this, CANDigitalInput.LimitSwitch.kReverse, polarity);
  }

  @Override
  public CANEncoder getEncoder(EncoderType sensorType, int counts_per_rev) {
    return new SimmableCANEncoder(this, sensorType, counts_per_rev);
  }

}