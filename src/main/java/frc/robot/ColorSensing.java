package frc.robot;

import java.awt.Color;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C.Port;

public class ColorSensing {
  public float[] hsv = new float[3];
  public int red, green, blue;
  final float tolerance = 0.025f;
  final float kYellow = 0.25f;
  final float kRed = 0.10f;
  final float kGreen = 0.35f;
  final float kCyan = 0.50f;
  float mindiff = 1f;
  float diff;
  String color = "Error";

  public ColorSensing() {

  }
}