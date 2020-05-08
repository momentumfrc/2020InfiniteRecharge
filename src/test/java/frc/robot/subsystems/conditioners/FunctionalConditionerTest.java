package frc.robot.subsystems.conditioners;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class FunctionalConditionerTest {
  private static final double ASSERT_DELTA = 1e-5d;

  @Test
  public void test1() {
    DriveConditioner conditioner = new FunctionalConditioner((in) -> 2 * in, (in) -> 0.5 * in);

    assertEquals(0, conditioner.conditionMove(0), ASSERT_DELTA);
    assertEquals(0, conditioner.conditionTurn(0), ASSERT_DELTA);

    assertEquals(1, conditioner.conditionMove(0.5), ASSERT_DELTA);
    assertEquals(0.25, conditioner.conditionTurn(0.5), ASSERT_DELTA);

    assertEquals(-1, conditioner.conditionMove(-0.5), ASSERT_DELTA);
    assertEquals(-0.25, conditioner.conditionTurn(-0.5), ASSERT_DELTA);
  }

  @Test
  public void test2() {
    DriveConditioner conditioner = new FunctionalConditioner((in) -> in - 1, (in) -> in + 1);

    assertEquals(-1, conditioner.conditionMove(0), ASSERT_DELTA);
    assertEquals(1, conditioner.conditionTurn(0), ASSERT_DELTA);

    assertEquals(-0.5, conditioner.conditionMove(0.5), ASSERT_DELTA);
    assertEquals(1.5, conditioner.conditionTurn(0.5), ASSERT_DELTA);

    assertEquals(-1.5, conditioner.conditionMove(-0.5), ASSERT_DELTA);
    assertEquals(0.5, conditioner.conditionTurn(-0.5), ASSERT_DELTA);
  }
}