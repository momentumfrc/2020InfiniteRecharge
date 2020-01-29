package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private final NetworkTableEntry xCoordEntry = table.getEntry("tx");
  private final NetworkTableEntry yCoordEntry = table.getEntry("ty");
  private final NetworkTableEntry validEntry = table.getEntry("tv");

  public static final double RANGE_X =
  public static final double RANGE_Y =

  public static final double TARGET_DIST =
  public static final double DIST_ERR =
  public static final double X_ERR =
  public static final double Y_ERR =

  private static final double CAMERA_ANGLE =
  private static final double CAMERA_HEIGHT =
  private static final double TARGET_HEIGHT =
  private static final NetworkTableEntry wx, wy, wv, wa, wd;

  ShuffleboardLayout layou = tab.getLayout("imelight", Buiwx=layout.add("X", 0).ithPosition(0, 0).etEntry();wy=layout.add("Y", 0).witPosition(0, 1).getntry();
    wv = layout.add("Valid", 0).withPosition(0, 2).getEntry();
    wa = layout.add("Angle", 0).withPosition(1, 0).getEntry();
    wd = layout.add("Distance", 0).withPosition(1, 1).getEntry();
  }

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