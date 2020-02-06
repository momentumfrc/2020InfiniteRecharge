package frc.robot.subsystems;

import java.awt.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ColorOptions;

/**
 * A class to facilitate getting one of four colors from the REVRobotics
 * ColorSensorV3
 */
public class ColorSubsystem extends SubsystemBase {
  // The array that is used to contain the output of java.awt.Color.RGBtoHSB()
  private final float[] hsv = new float[3];
  // The maximum allowed absolute distance from the constant to ensure an accurate
  // reading
  private final float tolerance = 0.025f;
  // The constants that correspond to the average hue (H) of the Control Panel
  // colors.
  private final float kYellow = 0.25f;
  private final float kRed = 0.10f;
  private final float kGreen = 0.35f;
  private final float kCyan = 0.50f;
  // Used to prevent repeated indexing of hsv[]
  private float hue;
  // A number used to store the minimum difference between the measured value and
  // the constants
  private float mindiff = 1f;
  // The absolute difference between the measured value and the constants
  private float diff;

  // The REVRobotics ColorSensorV3
  private final ColorSensorV3 colorSensor = new ColorSensorV3(Port.kOnboard);

  public void ColorSensing() {

  }

  /**
   * 
   * @return One of four colors on the Control Panel, NONE, or ERROR. Returns one
   *         of the four colors if in range, NONE if not close enough to any
   *         color, and ERROR otherwise.
   */
  public ColorOptions getColor() {
    // Sets the return string to Error so it is returned if no conditions are
    // fulfilled
    // Used to store the return message
    ColorOptions color = ColorOptions.ERROR;
    // The ints used to store the raw ADC output of the ColorSensorV3
    int red, green, blue;
    RawColor rgb = colorSensor.getRawColor();
    // Gets the raw ADC values
    red = rgb.red;
    green = rgb.green;
    blue = rgb.blue;
    // Converts to HSV
    Color.RGBtoHSB(red, green, blue, hsv);
    hue = hsv[0];
    mindiff = 1f;
    // Gets the difference between the measured value and the first constant.
    diff = Math.abs(hue - kYellow);
    // Checks if the current difference is smaller than the previous difference.
    // This will narrow down to the closest color constant over the 4 if statements.
    // Setting mindiff to diff ensures that the closest constant will always be
    // picked at the end, no matter the order.
    if (diff < mindiff) {
      mindiff = diff;
      color = ColorOptions.YELLOW;
    }
    diff = Math.abs(hue - kRed);
    if (diff < mindiff) {
      mindiff = diff;
      color = ColorOptions.RED;
    }
    diff = Math.abs(hue - kGreen);
    if (diff < mindiff) {
      mindiff = diff;
      color = ColorOptions.GREEN;
    }
    diff = Math.abs(hue - kCyan);
    if (diff < mindiff) {
      mindiff = diff;
      color = ColorOptions.CYAN;
    }
    // If the smallest difference from any constant is greater than the tolerance,
    // return None.
    if (mindiff > tolerance)
      color = ColorOptions.NONE;
    return color;
  }

  @Override
  public void periodic() {

  }
}