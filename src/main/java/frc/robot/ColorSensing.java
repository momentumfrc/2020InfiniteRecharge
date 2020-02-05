package frc.robot;

import java.awt.Color;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C.Port;

/**
 * A class to facilitate getting one of four colors from the REVRobotics
 * ColorSensorV3
 */
public class ColorSensing {
  /**
   * @param hsv The array that is used to contain the output of
   *            java.awt.Color.RGBtoHSB()
   */
  public float[] hsv = new float[3];
  // The ints used to store the raw ADC output of the ColorSensorV3
  public int red, green, blue;
  // The maximum allowed absolute distance from the constant to ensure an accurate
  // reading
  final float tolerance = 0.025f;
  // The constants that correspond to the average hue (H) of the Control Panel
  // colors.
  final float kYellow = 0.25f;
  final float kRed = 0.10f;
  final float kGreen = 0.35f;
  final float kCyan = 0.50f;
  // Used to prevent repeated indexing of hsv[]
  private float hue;
  // A number used to store the minimum difference between the measured value and
  // the constants
  private float mindiff = 1f;
  // The absolute difference between the measured value and the constants
  private float diff;
  // Used to store the return message
  String color;
  // The REVRobotics ColorSensorV3
  final ColorSensorV3 colorSensor;

  public ColorSensing() {
    // Initializes the colorSensor with the onboard I^2C port on the roboRIO
    colorSensor = new ColorSensorV3(Port.kOnboard);
  }

  public String getColor() {
    // Sets the return string to Error so it is returned if no conditions are
    // fulfilled
    color = "Error";
    // Gets the raw ADC values
    red = colorSensor.getRed();
    green = colorSensor.getGreen();
    blue = colorSensor.getBlue();
    // Converts to HSV
    Color.RGBtoHSB(red, green, blue, hsv);
    hue = hsv[0];
    mindiff = 1f;
    // Gets the difference between the measured value and the first constant
    diff = Math.abs(hue - kYellow);
    // Checks if the difference is in range
    if (diff < mindiff) {
      mindiff = diff;
      color = "Yellow";
    }
    // Gets the difference between the measured value and the second constant
    diff = Math.abs(hue - kRed);
    // Checks if the difference is in range
    if (diff < mindiff) {
      mindiff = diff;
      color = "Red";
    }
    // Gets the difference between the measured value and the third constant
    diff = Math.abs(hue - kGreen);
    // I don't really need to tell you what this does, do I?
    if (diff < mindiff) {
      mindiff = diff;
      color = "Green";
    }
    // I really don't need to tell you what this does.
    diff = Math.abs(hue - kCyan);
    // I'm not even gonna bother--well, I guess... this checks if the diff is in
    // range. Happy now?
    if (diff < mindiff) {
      mindiff = diff;
      color = "Cyan";
    }
    // If the smallest difference from any constant is greater than the tolerance,
    // return None.
    if (mindiff > tolerance)
      color = "None";
    return color;
  }
}