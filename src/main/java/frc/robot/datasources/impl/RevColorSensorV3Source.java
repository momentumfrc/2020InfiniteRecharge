package frc.robot.datasources.impl;

import frc.robot.datasources.ColorSource;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;
import edu.wpi.first.wpilibj.I2C.Port;

class RevColorSensorV3Source implements ColorSource {

  // The REVRobotics ColorSensorV3
  private final ColorSensorV3 colorSensor = new ColorSensorV3(Port.kOnboard);

  public RawColor getColor() {
    return colorSensor.getRawColor();
  }
}