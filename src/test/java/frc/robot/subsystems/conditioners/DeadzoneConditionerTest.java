package frc.robot.subsystems.conditioners;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class DeadzoneConditionerTest {
  private static final double ASSERT_DELTA = 1e-5d;

  @Test
  public void test() {
    DriveConditioner conditioner = new DeadzoneConditioner();

    assertEquals(0, conditioner.conditionMove(0), ASSERT_DELTA);
    assertEquals(0, conditioner.conditionTurn(0), ASSERT_DELTA);

    assertEquals(0, conditioner.conditionMove(0.05), ASSERT_DELTA);
    assertEquals(0, conditioner.conditionTurn(0.05), ASSERT_DELTA);

    assertEquals(0, conditioner.conditionMove(-0.05), ASSERT_DELTA);
    assertEquals(0, conditioner.conditionTurn(-0.05), ASSERT_DELTA);

    assertEquals(0, conditioner.conditionMove(0.1), ASSERT_DELTA);
    assertEquals(0, conditioner.conditionTurn(0.1), ASSERT_DELTA);

    assertEquals(0, conditioner.conditionMove(-0.1), ASSERT_DELTA);
    assertEquals(0, conditioner.conditionTurn(-0.1), ASSERT_DELTA);

    assertEquals(1.0 / 9.0d, conditioner.conditionMove(0.2), ASSERT_DELTA);
    assertEquals(1.0 / 9.0d, conditioner.conditionTurn(0.2), ASSERT_DELTA);

    assertEquals(-1.0 / 9.0d, conditioner.conditionMove(-0.2), ASSERT_DELTA);
    assertEquals(-1.0 / 9.0d, conditioner.conditionTurn(-0.2), ASSERT_DELTA);

    assertEquals(1, conditioner.conditionMove(1), ASSERT_DELTA);
    assertEquals(1, conditioner.conditionTurn(1), ASSERT_DELTA);

    assertEquals(-1, conditioner.conditionMove(-1), ASSERT_DELTA);
    assertEquals(-1, conditioner.conditionTurn(-1), ASSERT_DELTA);
  }
}