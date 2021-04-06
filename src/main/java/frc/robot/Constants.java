/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpiutil.math.numbers.N2;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Falcon CAN addresses
  public static final int FALCON_DRIVE_LEFT_FRONT_CAN_ADDR = 3;
  public static final int FALCON_DRIVE_LEFT_REAR_CAN_ADDR = 4;
  public static final int FALCON_DRIVE_RIGHT_FRONT_CAN_ADDR = 1;
  public static final int FALCON_DRIVE_RIGHT_REAR_CAN_ADDR = 2;

  // SparkMAX CAN addresses
  public static final int SPARKMAX_SHOOTER_CAN_ADDR_RIGHT = 14;
  public static final int SPARKMAX_SHOOTER_CAN_ADDR_LEFT = 10;
  public static final int SPARKMAX_SHOOTER_HOOD_CAN_ADDR = 3;

  // Pneumatic solenoid addresses
  public static final int INTAKE_PISTON_PCM_CHAN_DEPLOY = 0;
  public static final int INTAKE_PISTON_PCM_CHAN_STOW = 1;

  // Brushed PWM addresses
  public static final int INTAKE_VICTORSP_PWM_CHAN = 0;
  public static final int INTAKE_VICTORSP_PWM_CHAN_2 = 1;
  public static final int STORAGE_VICTORSP_PWM_CHAN = 2;
  public static final int SHOOTER_VICTORSP_PWM_CHAN = 3;
  public static final int CLIMBER_VICTORSP_PWM_CHAN = 4;
  public static final int ADDRESSABLE_LED_PWM_ADDRESS = 5;

  // PDP Channels
  public static final int INTAKE_VICTORSP_PDP_CHAN = 2;
  public static final int STORAGE_VICTORSP_PDP_CHAN = 4;

  // DIO channels
  public static final int CLIMBER_LIMIT_SWITCH = 0;
  public static final int CLIMBER_ENCODER_A_CHAN = 1;
  public static final int CLIMBER_ENCODER_B_CHAN = 2;

  // LEDs
  public static final int ADDRESSABLE_LED_LENGTH = 50;

  // Limelight constants
  public static final double CAMERA_ANGLE = 34.8;
  public static final double CAMERA_HEIGHT = 23.6;
  public static final double TARGET_HEIGHT = 84;
  public static final double DISTANCE_ERR_CORRECTION = 0.9304;

  public static final double ksVolts = 0.506;
  public static final double kvVoltSecondsPerMeter = 2.37;
  public static final double kaVoltSecondsSquaredPerMeter = 0.246;

  public static final double kvVoltSecondsPerRadian = 2.37;
  public static final double kaVoltSecondsSquaredPerRadian = 0.246;

  public static final LinearSystem<N2, N2, N2> kDrivetrainPlant = LinearSystemId.identifyDrivetrainSystem(
      kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter, kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian);
}
