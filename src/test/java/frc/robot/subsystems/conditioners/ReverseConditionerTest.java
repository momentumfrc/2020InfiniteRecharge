package frc.robot.subsystems.conditioners;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class ReverseConditionerTest {

  private static final double ASSERT_DELTA = 1e-5d;

  @Test
  public void test() {
    ReverseConditioner conditioner = new ReverseConditioner();
    conditioner.setReversed(false);
    assertEquals(false, conditioner.isReversed());

    assertEquals(0, conditioner.conditionMove(0), ASSERT_DELTA);
    assertEquals(0, conditioner.conditionTurn(0), ASSERT_DELTA);

    assertEquals(0.5, conditioner.conditionMove(0.5), ASSERT_DELTA);
    assertEquals(0.5, conditioner.conditionTurn(0.5), ASSERT_DELTA);

    assertEquals(-0.5, conditioner.conditionMove(-0.5), ASSERT_DELTA);
    assertEquals(-0.5, conditioner.conditionTurn(-0.5), ASSERT_DELTA);

    conditioner.setReversed(true);
    assertEquals(true, conditioner.isReversed());

    assertEquals(0, conditioner.conditionMove(0), ASSERT_DELTA);
    assertEquals(0, conditioner.conditionTurn(0), ASSERT_DELTA);

    assertEquals(-0.5, conditioner.conditionMove(0.5), ASSERT_DELTA);
    assertEquals(0.5, conditioner.conditionTurn(0.5), ASSERT_DELTA);

    assertEquals(0.5, conditioner.conditionMove(-0.5), ASSERT_DELTA);
    assertEquals(-0.5, conditioner.conditionTurn(-0.5), ASSERT_DELTA);

    conditioner.toggleReversed();
    assertEquals(false, conditioner.isReversed());

    assertEquals(0, conditioner.conditionMove(0), ASSERT_DELTA);
    assertEquals(0, conditioner.conditionTurn(0), ASSERT_DELTA);

    assertEquals(0.5, conditioner.conditionMove(0.5), ASSERT_DELTA);
    assertEquals(0.5, conditioner.conditionTurn(0.5), ASSERT_DELTA);

    assertEquals(-0.5, conditioner.conditionMove(-0.5), ASSERT_DELTA);
    assertEquals(-0.5, conditioner.conditionTurn(-0.5), ASSERT_DELTA);

    conditioner.toggleReversed();
    assertEquals(true, conditioner.isReversed());

    assertEquals(0, conditioner.conditionMove(0), ASSERT_DELTA);
    assertEquals(0, conditioner.conditionTurn(0), ASSERT_DELTA);

    assertEquals(-0.5, conditioner.conditionMove(0.5), ASSERT_DELTA);
    assertEquals(0.5, conditioner.conditionTurn(0.5), ASSERT_DELTA);

    assertEquals(0.5, conditioner.conditionMove(-0.5), ASSERT_DELTA);
    assertEquals(-0.5, conditioner.conditionTurn(-0.5), ASSERT_DELTA);
  }
}