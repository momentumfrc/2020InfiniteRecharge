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
    System.out.format("rc: Red:%d Green:%d Blue:%d\n", rc.red, rc.green, rc.blue);

    ColorSubsystem cs = new ColorSubsystem(new MockColorSource(rc));

    ColorOptions co = cs.getColor();
    System.out.format("co: %s\n", co.toString());

    assertEquals(ColorOptions.RED, co);
  }
}
