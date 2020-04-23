package frc.robot.subsystems.conditioners;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class SpeedLimitConditionerTest {

  private static final double ASSERT_DELTA = 1e-5d;

  @Test
  public void test() {
    SpeedLimitConditioner conditioner = new SpeedLimitConditioner();

    assertEquals(0, conditioner.conditionMove(0), ASSERT_DELTA);
    assertEquals(0, conditioner.conditionTurn(0), ASSERT_DELTA);

    assertEquals(0.5, conditioner.conditionMove(0.5), ASSERT_DELTA);
    assertEquals(0.5, conditioner.conditionTurn(0.5), ASSERT_DELTA);

    assertEquals(-0.5, conditioner.conditionMove(-0.5), ASSERT_DELTA);
    assertEquals(-0.5, conditioner.conditionTurn(-0.5), ASSERT_DELTA);

    conditioner.decSpeedLimit();
    conditioner.decSpeedLimit();
    conditioner.decSpeedLimit();

    assertEquals(0, conditioner.conditionMove(0), ASSERT_DELTA);
    assertEquals(0, conditioner.conditionTurn(0), ASSERT_DELTA);

    assertEquals(0.25, conditioner.conditionMove(0.5), ASSERT_DELTA);
    assertEquals(0.5, conditioner.conditionTurn(0.5), ASSERT_DELTA);

    assertEquals(-0.25, conditioner.conditionMove(-0.5), ASSERT_DELTA);
    assertEquals(-0.5, conditioner.conditionTurn(-0.5), ASSERT_DELTA);

    conditioner.incSpeedLimit();
    conditioner.incSpeedLimit();
    conditioner.incSpeedLimit();

    assertEquals(0, conditioner.conditionMove(0), ASSERT_DELTA);
    assertEquals(0, conditioner.conditionTurn(0), ASSERT_DELTA);

    assertEquals(0.5, conditioner.conditionMove(0.5), ASSERT_DELTA);
    assertEquals(0.5, conditioner.conditionTurn(0.5), ASSERT_DELTA);

    assertEquals(-0.5, conditioner.conditionMove(-0.5), ASSERT_DELTA);
    assertEquals(-0.5, conditioner.conditionTurn(-0.5), ASSERT_DELTA);
  }
}