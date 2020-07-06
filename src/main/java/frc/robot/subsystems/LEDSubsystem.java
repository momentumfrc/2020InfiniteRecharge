package frc.robot.subsystems;

import org.usfirst.frc.team4999.lights.AddressableLEDDisplay;
import org.usfirst.frc.team4999.lights.AnimationCoordinator;
import org.usfirst.frc.team4999.lights.Animator;
import org.usfirst.frc.team4999.lights.Color;
import org.usfirst.frc.team4999.lights.animations.*;

import edu.wpi.first.wpilibj.AddressableLED;

import static frc.robot.Constants.*;

public class LEDSubsystem {
  private AddressableLEDDisplay display;
  private Animator animator;
  private AnimationCoordinator coordinator;

  public AddressableLED test;

  private static final String BASE_ANIMATION_KEY = "BASE_ANIMATION";
  private static final int BASE_ANIMATION_PRIORITY = 0;

  private static Color[] rainbowcolors = { new Color(139, 0, 255), Color.BLUE, Color.GREEN, Color.YELLOW,
      new Color(255, 127, 0), Color.RED };

  public final Animation rainbow = new AnimationSequence(new Animation[] { Snake.rainbowSnake(70),
      Fade.rainbowFade(100, 20), new Bounce(Color.WHITE, rainbowcolors, 40, 50), new Stack(rainbowcolors, 50, 40),
      new BounceStack(rainbowcolors, 20, 40) }, new int[] { 5000, 5000, 10000, 10000, 10000 });

  public LEDSubsystem() {

    try {
      display = new AddressableLEDDisplay(ADDRESSABLE_LED_PWM_ADDRESS, ADDRESSABLE_LED_LENGTH);
      animator = new Animator(display);
      coordinator = new AnimationCoordinator(animator);

      setBaseAnimation(rainbow);
    } catch (Exception e) {
      e.printStackTrace();
      coordinator = null;
      if (animator != null) {
        animator.stopAnimation();
      }
      animator = null;
      if (display != null) {
        display.stop();
      }
      display = null;
    }

  }

  public void setBaseAnimation(Animation animation) {
    if (coordinator != null) {
      coordinator.popAnimation(BASE_ANIMATION_KEY);
      coordinator.pushAnimation(BASE_ANIMATION_KEY, animation, BASE_ANIMATION_PRIORITY, false);
    }
  }

}