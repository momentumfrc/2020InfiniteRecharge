package frc.robot.datasources;

import com.revrobotics.ColorSensorV3.RawColor;

public interface ColorSource {
  public RawColor getColor();
}