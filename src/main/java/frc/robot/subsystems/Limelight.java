
package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

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

  private NetworkTableEntry wx;
  private NetworkTableEntry wy;
  private NetworkTableEntry wv;
  private NetworkTableEntry wa;
  private NetworkTableEntry wd;

  private LimelightData lastData;

  public Limelight(ShuffleboardTab tab, ShuffleboardTab matchTab, int col, int row) {
    ShuffleboardLayout layout = tab.getLayout("Limelight", BuiltInLayouts.kGrid).withPosition(col, row).withSize(2, 3);
    wx = layout.add("X", 0).withPosition(0, 0).getEntry();
    wy = layout.add("Y", 0).withPosition(0, 1).getEntry();
    wv = layout.add("Valid", 0).withPosition(0, 2).getEntry();
    wa = layout.add("Angle", 0).withPosition(1, 0).getEntry();
    wd = layout.add("Distance", 0).withPosition(1, 1).getEntry();

    matchTab.add(new HttpCamera("Limelight", "http://10.49.99.11:5800/", HttpCameraKind.kMJPGStreamer))
        .withPosition(6, 0).withSize(3, 3).withProperties(Map.of("Show controls", false));
  }

  public LimelightData getData() {
    return lastData;
  }

  public void lightsOff() {
    table.getEntry("ledMode").setNumber(1);
  }

  public void lightsOn() {
    table.getEntry("ledMode").setNumber(3);
  }

  @Override
  public void periodic() {
    lastData = new LimelightData(validEntry.getDouble(0), xCoordEntry.getDouble(0), yCoordEntry.getDouble(0));
  }

  public class LimelightData {

    private final double xCoord;
    private final double yCoord;
    private final boolean validTarget;
    private final double dist;

    private LimelightData(double valid, double xAngle, double yAngle) {
      this.xCoord = xAngle;
      this.yCoord = yAngle;

      double height = Constants.TARGET_HEIGHT - Constants.CAMERA_HEIGHT;
      double slope = Math.tan(Math.toRadians(Constants.CAMERA_ANGLE + yAngle));
      if (slope > 0) {
        dist = height / slope * Constants.DISTANCE_ERR_CORRECTION;
      } else {
        dist = 0;
        valid = 0;
      }
      this.validTarget = valid == 1;

      wx.setDouble(xAngle);
      wy.setDouble(yAngle);
      wv.setDouble(valid);
      wa.setDouble(slope);
      wd.setDouble(dist);

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
        double x = xCoord();
        if (x <= 0.5) {
          return true;
        }
      }
      return false;
    }
  }
}
