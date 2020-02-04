/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.I2C.Port;
import com.revrobotics.ColorSensorV3;
import java.awt.Color;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private ColorSensorV3 colorsensor = new ColorSensorV3(Port.kOnboard);
  private float hsv[] = new float[3];
  private Timer timer = new Timer();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    Color.RGBtoHSB(255, 0, 0, hsv);
    System.out.println("RED: Hue: " + hsv[0] + " Saturation: " + hsv[1] + " Value: " + hsv[2]);
    Color.RGBtoHSB(0, 255, 0, hsv);
    System.out.println("GREEN: Hue: " + hsv[0] + " Saturation: " + hsv[1] + " Value: " + hsv[2]);
    Color.RGBtoHSB(0, 0, 255, hsv);
    System.out.println("BLUE: Hue: " + hsv[0] + " Saturation: " + hsv[1] + " Value: " + hsv[2]);

    timer.reset();
    timer.start();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    if (timer.hasPeriodPassed(1)) {
      int blue = colorsensor.getBlue();
      int green = colorsensor.getGreen();
      int red = colorsensor.getRed();

      Color.RGBtoHSB(red, green, blue, hsv);
      System.out.println("R: " + red + " G: " + green + " B: " + blue);
      System.out.println("Hue: " + hsv[0] + " Saturation: " + hsv[1] + " Value: " + hsv[2]);
      float hue = hsv[0];
      final float tolerance = 0.025f;
      final float kYellow = 0.25f;
      final float kRed = 0.10f;
      final float kGreen = 0.36f;
      final float kCyan = 0.50f;
      float mindiff = 1f;
      float diff;
      String color = "Error";

      diff = Math.abs(hue - kYellow);
      if (diff < mindiff) {
        mindiff = diff;
        color = "Yellow";
      }
      diff = Math.abs(hue - kRed);
      if (diff < mindiff) {
        mindiff = diff;
        color = "Red";
      }
      diff = Math.abs(hue - kGreen);
      if (diff < mindiff) {
        mindiff = diff;
        color = "Green";
      }
      diff = Math.abs(hue - kCyan);
      if (diff < mindiff) {
        mindiff = diff;
        color = "Cyan";
      }
      if (mindiff > tolerance) {
        color = "None";
      }
      System.out.println("mindiff: " + mindiff);
      System.out.println("Detected color: " + color);

    }
  }
}