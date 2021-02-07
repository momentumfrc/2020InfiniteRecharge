package frc.robot.subsystems;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class FalconDriveSubsystemTest {
  private final FalconDriveSubsystem m_subsystem;

  public FalconDriveSubsystemTest() {
    m_subsystem = new FalconDriveSubsystem(Shuffleboard.getTab("test"));
  }

  @Test
  public void testWheelVelocity() {
    // Makes sure that a lack of motion will not produce a nonzero wheel velocity.
    assertEquals(0, m_subsystem.getWheelVelocity(0), 0);

    // 987.5575 et/100ms should correspond to 1m/s in wheel speed
    double testvar = m_subsystem.getWheelVelocity(987.5575);
    assertEquals(1, testvar, 1e-2);
    System.out.println(testvar);
  }
}
