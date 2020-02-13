package frc.robot.subsystems;

import java.awt.Color;
import java.util.HashMap;

import com.revrobotics.ColorSensorV3.RawColor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.datasources.ColorSource;
import frc.robot.utils.ColorOptions;

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
  // A number used to store the minimum difference between the measured value and
  // the constants
  private float mindiff;

  // Sets the return string to Error so it is returned if no conditions are
  // fulfilled
  // Used to store the return message
  private ColorOptions color;

  // The REVRobotics ColorSensorV3
  private final ColorSource colorSource;

  private final HashMap<Float, ColorOptions> colorMap = new HashMap<>();

  public ColorSubsystem(ColorSource source) {
    colorSource = source;

    // The constants that correspond to the average hue (H) of the Control Panel
    // colors.
    final float kYellow = 0.25f;
    final float kRed = 0.10f;
    final float kGreen = 0.35f;
    final float kCyan = 0.50f;
    colorMap.put(kYellow, ColorOptions.YELLOW);
    colorMap.put(kRed, ColorOptions.RED);
    colorMap.put(kGreen, ColorOptions.GREEN);
    colorMap.put(kCyan, ColorOptions.CYAN);
  }

  /**
   * 
   * @return One of four colors on the Control Panel, NONE, or ERROR. Returns one
   *         of the four colors if in range, NONE if not close enough to any
   *         color, and ERROR otherwise.
   */
  public ColorOptions getColor() {
    // Used to prevent repeated indexing of hsv[]
    float hue;
    // The ints used to store the raw ADC output of the ColorSensorV3
    int red, green, blue;
    RawColor rgb = colorSource.getColor();
    // Gets the raw ADC values
    red = rgb.red;
    green = rgb.green;
    blue = rgb.blue;
    // Converts to HSV
    Color.RGBtoHSB(red, green, blue, hsv);
    hue = hsv[0];
    mindiff = 1f;
    color = ColorOptions.ERROR;

    colorMap.forEach((k, v) -> {
      // Gets the difference between the measured value and the first constant.
      // The absolute difference between the measured value and the constants
      float diff = Math.abs(hue - k);
      // Checks if the current difference is smaller than the previous difference.
      // This will narrow down to the closest color constant over the 4 if statements.
      // Setting mindiff to diff ensures that the closest constant will always be
      // picked at the end, no matter the order.
      if (diff < mindiff) {
        mindiff = diff;
        color = v;
      }
    });
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