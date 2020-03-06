package frc.robot.utils;

import java.util.Arrays;

import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimEnum;

public class SimmableCANSparkMax extends CANSparkMax {

  private SimDevice m_simDevice;
  private SimDouble m_simSpeed;
  private SimBoolean m_simInvert;

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

}