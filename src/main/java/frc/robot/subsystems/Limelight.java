package frc.robot.subsystems;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LimelightData;

public class Limelight extends SubsystemBase {

  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private final NetworkTableEntry xCoordEntry = table.getEntry("tx");
  private final NetworkTableEntry yCoordEntry = table.getEntry("ty");
  private final NetworkTableEntry validEntry = table.getEntry("tv");

}

public class LimelightData {

  private final double xCoord;
  private final double yCoord;
  private final boolean validTarget;
  private final double dist;

  private LimelightData(double valid, double xAngle, double yAngle) {
    this.xCoord = xAngle;
    this.yCoord = yAngle;
}