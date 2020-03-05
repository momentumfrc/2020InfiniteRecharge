package frc.robot.subsystems;

import org.junit.Test;

import frc.robot.datasources.ColorSource;
import com.revrobotics.ColorSensorV3.RawColor;
import frc.robot.utils.ColorOptions;

import org.junit.Before;
import static org.junit.Assert.*;

public class ColorSubsystemTest {

  class MockColorSource implements ColorSource {

    private RawColor rawColor;

    MockColorSource(RawColor rawColor) {
      this.rawColor = rawColor;
    }

    public RawColor getColor() {
      return rawColor;
    }
  }

  @Before
  public void setup() {

  }

  @Test
  public void test() {
    ColorSubsystem cs = new ColorSubsystem(new MockColorSource(new RawColor(0, 0, 0, 0)));

    ColorOptions co = cs.getColor();

    assertEquals(ColorOptions.NONE, co);
  }

  @Test
  public void testRed() {
    RawColor rc = new RawColor(255, 153, 0, 0);

    ColorSubsystem cs = new ColorSubsystem(new MockColorSource(rc));

    ColorOptions co = cs.getColor();

    assertEquals(ColorOptions.RED, co);
  }

  @Test
  public void testGreen() {
    RawColor rc = new RawColor(0, 255, 0, 0);

    ColorSubsystem cs = new ColorSubsystem(new MockColorSource(rc));

    ColorOptions co = cs.getColor();

    assertEquals(ColorOptions.GREEN, co);
  }

  @Test
  public void testCyan() {
    RawColor rc = new RawColor(0, 255, 255, 0);

    ColorSubsystem cs = new ColorSubsystem(new MockColorSource(rc));

    ColorOptions co = cs.getColor();

    assertEquals(ColorOptions.CYAN, co);
  }

  @Test
  public void testYellow() {
    RawColor rc = new RawColor(153, 255, 0, 0);
    ColorSubsystem cs = new ColorSubsystem(new MockColorSource(rc));

    ColorOptions co = cs.getColor();

    assertEquals(ColorOptions.YELLOW, co);
  }

  @Test
  public void testPurple() {
    RawColor rc = new RawColor(255, 0, 255, 0);
    ColorSubsystem cs = new ColorSubsystem(new MockColorSource(rc));

    ColorOptions co = cs.getColor();

    assertEquals(ColorOptions.NONE, co);
  }

  @Test
  public void testAquamarine() {
    RawColor rc = new RawColor(0, 255, 85, 0);
    ColorSubsystem cs = new ColorSubsystem(new MockColorSource(rc));

    ColorOptions co = cs.getColor();

    assertEquals(ColorOptions.NONE, co);
  }

  @Test
  public void testGold() {
    RawColor rc = new RawColor(229, 255, 0, 0);
    ColorSubsystem cs = new ColorSubsystem(new MockColorSource(rc));

    ColorOptions co = cs.getColor();

    assertEquals(ColorOptions.NONE, co);
  }

  @Test
  public void testRouge() {
    RawColor rc = new RawColor(255, 98, 0, 0);
    ColorSubsystem cs = new ColorSubsystem(new MockColorSource(rc));

    ColorOptions co = cs.getColor();

    assertEquals(ColorOptions.NONE, co);
  }
}
