
package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private final NetworkTableEntry xCoordEntry = table.getEntry("tx");
  private final NetworkTableEntry yCoordEntry = table.getEntry("ty");
  private final NetworkTableEntry validEntry = table.getEntry("tv");

  public static final double RANGE_X = 29.8;
  public static final double RANGE_Y = 24.85;

  public static final double TARGET_DIST = 3;
  public static final double DIST_ERR = 0;
  public static final double X_ERR = 0;
  public static final double Y_ERR = 0;

  private static final double CAMERA_ANGLE = 56.5;
  private static final double CAMERA_HEIGHT = 23.6;
  private static final double TARGET_HEIGHT = 84;
  private static NetworkTableEntry wx, wy, wv, wa, wd;

  public LimelightData getData() {
    return new LimelightData(validEntry.getDouble(0), xCoordEntry.getDouble(0), yCoordEntry.getDouble(0));
  }

  public class LimelightData {

    private final double xCoord;
    private final double yCoord;
    private final boolean validTarget;
    private final double dist;

    private LimelightData(double valid, double xAngle, double yAngle) {
      this.xCoord = xAngle;
      this.yCoord = yAngle;

      double height = TARGET_HEIGHT - CAMERA_HEIGHT;
      double slope = Math.tan(CAMERA_ANGLE + yAngle);
      if (slope > 0) {
        dist = height / slope;
      } else {
        dist = 0;
        valid = 0;
      }
      this.validTarget = valid == 1;

      wx.setDouble(xAngle);
      wy.setDouble(yAngle);
      wv.setDouble(valid);
      wa.setDouble(slope);
      wv.setDouble(dist);

      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }

    public double dist() {
      return dist;
    }

    public double xCoord() {
      return xCoord;
    }

    public double yCoord() {
      return yCoord;
    }

    public boolean valid() {
      return validTarget;
    }

    public boolean targetMet() {
      if (valid()) {
        double x = Math.abs(xCoord());
        double y = Math.abs(yCoord());
        double d = Math.abs(dist() - DIST_ERR);
        if (x <= X_ERR && y <= Y_ERR && d <= TARGET_DIST) {
          return true;
        }
      }
      return false;
    }
  }
}