/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import org.usfirst.frc.team4999.utils.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

public class FalconDriveSubsystem extends DriveSubsystem {
  // Need to use WPI_TalonFX so that DifferentialDrive will accept the motors.
  private final WPI_TalonFX leftFront = new WPI_TalonFX(FALCON_DRIVE_LEFT_FRONT_CAN_ADDR);
  private final WPI_TalonFX leftRear = new WPI_TalonFX(FALCON_DRIVE_LEFT_REAR_CAN_ADDR);
  private final WPI_TalonFX rightFront = new WPI_TalonFX(FALCON_DRIVE_RIGHT_FRONT_CAN_ADDR);
  private final WPI_TalonFX rightRear = new WPI_TalonFX(FALCON_DRIVE_RIGHT_REAR_CAN_ADDR);

  private final SensorCollection leftEnc = new SensorCollection(leftFront);
  private final SensorCollection rightEnc = new SensorCollection(rightFront);

  private final boolean pidEnabled = false;

  /**
   * @param ENC_TICKS_PER_METER The number of TalonFX encoder ticks per meter that
   *                            the robot drives on 6" wheels. Used to input ft/s
   *                            into WPI_TalonFX.set(VelocityControl, double
   *                            value), which takes its value in encoder ticks per
   *                            second.
   * 
   */
  private final double ENC_TICKS_PER_METER = 4278.215;
  /**
   * @param DRIVE_BASE_WIDTH_INCHES The number of inches between wheels on the
   *                                drive base of the robot.
   */
  private final double DRIVE_BASE_WIDTH_INCHES = 26;
  /**
   * @param SPEED_LIMIT_METERS_PER_S The maximum individual wheel speed of the
   *                                 robot in ft/s.
   */
  private final double SPEED_LIMIT_METERS_PER_S = 4;
  /**
   * @param TURN_LIMIT_RAD_PER_S The maximum safe angular velocity of the robot,
   *                             in radians per second.
   */
  private final double TURN_LIMIT_RAD_PER_S = 4 * Math.PI;
  /**
   * @param kP The Proportional Gain, used in PID to produce a linear curve.
   */
  private final double kP = 0;
  /**
   * @param kI The Integral Gain, used in PID to produce a sinoid curve.
   */
  private final double kI = 0;
  /**
   * @param kD The Differential Gain, used in PID to produce a parabolic curve.
   */
  private final double kD = 0;
  /**
   * @param kIz The Integral Zone, used in PID to control the maximum value of the
   *            integral accumulator.
   */
  private final int kIz = 0;
  /**
   * @param kF The Feed-Forward Gain, used in PID to anticipate future changes in
   *           error and stabilize a PID curve.
   */
  private final double kF = 1;

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
      Units.inchesToMeters(DRIVE_BASE_WIDTH_INCHES));

  public FalconDriveSubsystem() {
    // Invert one side of the robot
    // These should always be opposites
    // If the robot drives backwards, flip both
    leftFront.setInverted(true);
    leftRear.setInverted(true);
    rightFront.setInverted(false);

    // Set the braking mode
    leftFront.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);

    // Slaves the left rear motor to the left front motor
    leftRear.follow(leftFront);
    rightRear.follow(rightFront);
    // Sets the PID configs for all motors.
    leftFront.config_kP(0, kP);
    leftFront.config_kI(0, kI);
    leftFront.config_kD(0, kD);
    leftFront.config_IntegralZone(0, kIz);
    leftFront.config_kF(0, kF);
    rightFront.config_kP(0, kP);
    rightFront.config_kI(0, kI);
    rightFront.config_kD(0, kD);
    rightFront.config_IntegralZone(0, kIz);
    rightFront.config_kF(0, kF);
  }

  /**
   * @param moveRequest The "requested" forward motion as given by the controller
   *                    joystick, from -1 to 1.
   * @param turnRequest The "requested" angular motion as given by the controller
   *                    joystick, from -1 to 1.
   */
  public void drive(final double moveRequest, final double turnRequest) {
    if (pidEnabled) {
      /**
       * Since moveRequest is from -1 to 1 and we need a value in meters per second to
       * feed to ChassisSpeeds, we scale the moveRequest to the speed limit as
       * converted to ft/s.
       */
      final double moveReqScaled = Utils.map(moveRequest, -1, 1, -SPEED_LIMIT_METERS_PER_S, SPEED_LIMIT_METERS_PER_S);
      /**
       * Since turnRequest is from -1 to 1 and we need a value in radians per second
       * to feed to ChassisSpeeds, we scale the turnRequest to the angular velocity
       * limit in radians/second.
       */
      final double turnReqScaled = Utils.map(turnRequest, -1, 1, -TURN_LIMIT_RAD_PER_S, TURN_LIMIT_RAD_PER_S);
      /**
       * The object that handles the calculations for how fast each side of the robot
       * should drive to accomplish the scaled move and turn requests.
       */
      final var chassisSpeeds = new ChassisSpeeds(moveReqScaled, 0, turnReqScaled);
      /**
       * The object that converts the overall chassis speed into individual wheel
       * speeds.
       */
      final DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
      /**
       * The variables that store the outputs of the wheel speeds, in meters per
       * second.
       */
      final double leftMPerS = wheelSpeeds.leftMetersPerSecond;
      final double rightMPerS = wheelSpeeds.rightMetersPerSecond;
      /**
       * The variables that store the wheel speeds in TalonFX encoder ticks per
       * second, as converted from m/s to ft/s.
       */
      final double leftETPerS = metersToEncTicks(leftMPerS);
      final double rightETPerS = metersToEncTicks(rightMPerS);
      leftFront.set(ControlMode.Velocity, leftETPerS);
      rightFront.set(ControlMode.Velocity, rightETPerS);
    } else {
      leftFront.set(ControlMode.PercentOutput, Utils.clip(moveRequest + turnRequest, -1, 1));
      rightFront.set(ControlMode.PercentOutput, Utils.clip(moveRequest + turnRequest, -1, 1));
    }
  }

  public void resetEncoders() {
    leftEnc.setQuadraturePosition(0, 0); // Resets both encoders so they count from zero.
    rightEnc.setQuadraturePosition(0, 0);
  }

  /**
   * Stops both motors.
   */
  public void stop() {
    leftFront.stopMotor();
    rightFront.stopMotor();
  }

  /**
   * 
   * @param ft The number of meters to be converted to encoder ticks.
   * @return the number of encoder ticks that correspond to @param ft , assuming
   *         6-inch wheels.
   */
  private double metersToEncTicks(final double m) {
    return m * ENC_TICKS_PER_METER;
  }
}
