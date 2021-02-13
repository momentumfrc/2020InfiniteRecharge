package frc.robot.subsystems;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import edu.wpi.first.wpilibj.geometry.Pose2d;

public class FalconDriveSubsystemTest {
  private final FalconDriveSubsystem m_subsystem;

  private static final double TEST_DELTA = 1e-2;

  public FalconDriveSubsystemTest() {
    m_subsystem = new FalconDriveSubsystem(null);
  }

  @Test
  public void testWheelVelocity() {
    // Makes sure that a lack of motion will not produce a nonzero wheel velocity.
    assertEquals(0, m_subsystem.getWheelVelocity(0), 0);

    // 2048 et/100ms should correspond to ~0.445m/s in wheel speed
    double testvar = m_subsystem.getWheelVelocity(2048);
    assertEquals(0.445, testvar, TEST_DELTA);
  }

  @Test
  public void testPoseGen() {
    // Represents the robot turning in place
    Pose2d pose = m_subsystem.generatePose(2048, -2048);

    // Shouldn't be moving forward
    assertEquals(0, pose.getX(), TEST_DELTA);

    // Robot shouldn't be able to slide sideways
    assertEquals(0, pose.getY(), TEST_DELTA);

    // Rotation should equate to about 2.7 radians per second
    assertEquals(2.698, pose.getRotation().getRadians(), TEST_DELTA);
  }
}
