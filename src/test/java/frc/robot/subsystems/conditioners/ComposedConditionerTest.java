package frc.robot.subsystems.conditioners;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class ComposedConditionerTest {
  private static final double ASSERT_DELTA = 1e-5d;

  private DriveConditioner cond1 = new FunctionalConditioner((in) -> in + 0.2, (in) -> in + 0.2);
  private DriveConditioner cond2 = new FunctionalConditioner((in) -> -1 * in, (in) -> -1 * in);
  private DriveConditioner cond3 = new FunctionalConditioner((in) -> in * in, (in) -> in * in);
  private DriveConditioner cond4 = new FunctionalConditioner((in) -> in + 0.1, (in) -> in - 0.1);

  @Test
  public void testSingle() {
    DriveConditioner conditioner = new ComposedConditioner(cond1);
    assertEquals(0.2, conditioner.conditionMove(0), ASSERT_DELTA);
    assertEquals(0.2, conditioner.conditionTurn(0), ASSERT_DELTA);

    assertEquals(-0.8, conditioner.conditionMove(-1), ASSERT_DELTA);
    assertEquals(0.9, conditioner.conditionTurn(0.7), ASSERT_DELTA);
  }

  @Test
  public void testMultiple() {
    DriveConditioner conditioner = new ComposedConditioner(cond1, cond2, cond3, cond4);

    assertEquals(0.14, conditioner.conditionMove(0), ASSERT_DELTA);
    assertEquals(-0.06, conditioner.conditionTurn(0), ASSERT_DELTA);

    assertEquals(0.74, conditioner.conditionMove(-1), ASSERT_DELTA);
    assertEquals(0.71, conditioner.conditionTurn(0.7), ASSERT_DELTA);
  }
}